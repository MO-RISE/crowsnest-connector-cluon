# crowsnest-connector-cluon-lidar

A connector to a cluon-based UDP multicast setup for listening in on lidar data, unpack, transform and convey to a mqtt broker.

## Development setup

To setup the development environment:

```bach
  python3 -m venv env
  source env/bin/activate
```

Install everything thats needed for development:

```bach
  pip install -r requirements_dev.txt
```

In addition, code for `brefv` must be generated using the following commands:

```bach
    mkdir brefv
    datamodel-codegen --input brefv-spec/envelope.json --input-file-type jsonschema --output brefv/envelope.py
    datamodel-codegen --input brefv-spec/messages --input-file-type jsonschema  --reuse-model --output brefv/messages
```

To run the linters:

```bach
    black main.py tests
    pylint main.py
```

To run the tests:

```bach
    python -m pytest --verbose tests
```

## License

Apache 2.0, see [LICENSE](./LICENSE)