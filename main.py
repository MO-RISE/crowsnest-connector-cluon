"""Main entrypoint for this application"""
import os
import json
from typing import List
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from streamz import Stream, combine_latest

from paho.mqtt.client import Client as MQTT

from pycluon import OD4Session, Envelope
from pycluon.importer import import_odvd

import geojson

import lidar_utils as lu

## Cluon setup
THIS_DIR = Path(__file__).parent
brefv = import_odvd(THIS_DIR / "brefv" / "morise-brefv.odvd")
opendlv = import_odvd(THIS_DIR / "opendlv.standard-message-set" / "opendlv.odvd")

CLUON_CID = int(os.environ.get("CLUON_CID", 111))
CLUON_ENVELOPES_PER_REVOLUTION = int(
    os.environ.get("CLUON_ENVELOPES_PER_REVOLUTION", 2)
)
CLUON_SENSOR_ORIENTATION_X = int(os.environ.get("CLUON_SENSOR_ORIENTATION_X", 0))
CLUON_SENSOR_ORIENTATION_Y = int(os.environ.get("CLUON_SENSOR_ORIENTATION_Y", 0))
CLUON_SENSOR_ORIENTATION_Z = int(os.environ.get("CLUON_SENSOR_ORIENTATION_Z", 0))


## MQTT setup
MQTT_BROKER_HOST = os.environ.get("MQTT_BROKER_HOST")
MQTT_BROKER_PORT = int(os.environ.get("MQTT_BROKER_PORT", "1883"))
MQTT_CLIENT_ID = os.environ.get("MQTT_CLIENT_ID", "")
MQTT_TRANSPORT = os.environ.get("MQTT_TRANSPORT", "tcp")
MQTT_USER = os.environ.get("MQTT_USER")
MQTT_PASSWORD = os.environ.get("MQTT_PASSWORD")
MQTT_BASE_TOPIC = os.environ.get("MQTT_BASE_TOPIC")

mq = MQTT(client_id=MQTT_CLIENT_ID, transport=MQTT_TRANSPORT)
mq.username_pw_set(MQTT_USER, MQTT_PASSWORD)
mq.tls_set()
mq.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT)


### Point cloud handling

pcd_source: Stream = Stream()


def point_cloud_reading_extractor(envelopes: List[Envelope]) -> np.ndarray:
    """Extract a point cloud in numpy format from an odvd envelope

    Args:
        envelope (Envelope): Envelope containing a PointCloudReading

    Returns:
        np.ndarray: The point cloud as a 2D np.ndarray with shape (n, 3)
    """
    all_azimuths = []
    all_binary_distances = bytes()

    try:
        for envelope in envelopes:
            msg = opendlv.opendlv_proxy_PointCloudReading()
            msg.ParseFromString(envelope.serialized_data)
            start_azimuth = msg.startAzimuth
            end_azimuth = msg.endAzimuth
            entries_per_azimuth = msg.entriesPerAzimuth
            binary_distances = msg.distances

            # Bail early
            if not binary_distances:
                continue

            n_points = lu.get_number_of_points(binary_distances)
            n_azimuths = lu.get_number_of_azimuths(n_points, entries_per_azimuth)
            azimuths = np.linspace(
                start_azimuth, end_azimuth, int(n_azimuths), endpoint=True
            )

            all_azimuths.extend(azimuths.tolist())
            all_binary_distances += binary_distances

        points = lu.get_points(
            np.deg2rad(all_azimuths),
            np.deg2rad(lu.VERTICAL_ANGLES_16),
            all_binary_distances,
        )

        if not points.size:
            return None

        return points
    except Exception as exc:  # pylint: disable=broad-except
        print(exc)
        return None


pcd_extractor: Stream = (
    pcd_source.partition(CLUON_ENVELOPES_PER_REVOLUTION)
    .map(point_cloud_reading_extractor)
    .filter(lambda x: x is not None)
    .latest()
    .rate_limit(0.5)
)

### Heading handling
hdg_source: Stream = Stream()


def heading_extractor(envelope: Envelope):
    """Extract the heading from an odvd envelope

    Args:
        envelope (Envelope): Envelope containing a GeodeticHeadingReading

    Returns:
        float: Heading in degrees
    """
    msg = opendlv.opendlv_proxy_GeodeticHeadingReading()
    msg.ParseFromString(envelope.serialized_data)
    return msg.northHeading


hdg_extractor: Stream = hdg_source.map(heading_extractor)


## Position handling
pos_source: Stream = Stream()


def position_extractor(envelope: Envelope):
    """Extract the position from an odvd envelope

    Args:
        envelope (Envelope): Envelope containing a GeodeticWgs84Reading

    Returns:
        tuple: Position as (lon, lat)
    """
    msg = opendlv.opendlv_proxy_GeodeticWgs84Reading()
    msg.ParseFromString(envelope.serialized_data)
    return (msg.longitude, msg.latitude)


pos_extractor: Stream = pos_source.map(position_extractor)

# Joining
combined: Stream = combine_latest(
    pos_extractor,
    hdg_extractor,
    pcd_extractor,
    emit_on=pcd_extractor,
)

# Transform to correct coordinate system
def transform_coords(data: tuple):
    """Transform point cloud according to attitude of vessel and sensor mounting position

    Args:
        data (tuple): Tuple of data (position, heading, point cloud)

    Returns:
        tuple: (position, rotated point cloud)
    """
    pos, hdg, points = data
    orientation = [
        hdg + CLUON_SENSOR_ORIENTATION_Z,
        CLUON_SENSOR_ORIENTATION_Y,
        CLUON_SENSOR_ORIENTATION_X,
    ]
    transform = Rotation.from_euler("zyx", orientation, degrees=True)
    points = transform.apply(points)
    return pos, points


processed: Stream = combined.map(transform_coords)  # pylint: disable=no-member

# Push to mqtt
def to_mqtt(data):
    """Push data to a mqtt topic

    Args:
        data (tuple): (position, point cloud)
    """
    pos, points = data
    origin = geojson.Point(pos)
    mq.publish(
        f"{MQTT_BASE_TOPIC}/lidar",
        json.dumps({"origin": origin, "points": points.tolist()}),
    )


processed.sink(to_mqtt)


if __name__ == "__main__":
    print("All setup done, lets start processing messages!")

    # Register triggers
    session = OD4Session(CLUON_CID)
    session.add_data_trigger(49, pcd_source.emit)
    session.add_data_trigger(1051, hdg_source.emit)
    session.add_data_trigger(19, pos_source.emit)

    mq.loop_forever()
