FROM python:3.8-slim-bullseye

COPY requirements.txt requirements.txt

RUN pip3 install -r requirements.txt

WORKDIR /app

COPY lidar_utils.py lidar_utils.py
COPY main.py main.py

CMD ["python3", "main.py"]