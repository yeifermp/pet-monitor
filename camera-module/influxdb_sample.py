import os
from influxdb_client_3 import InfluxDBClient3, InfluxDBError, Point, WritePrecision, WriteOptions, write_client_options

host = os.getenv('INFLUX_HOST')
token = os.getenv('INFLUX_TOKEN')
database = os.getenv('INFLUX_DATABASE')

points = [Point("home").tag("room", "kitchen").field("temp",25.3).field("temp", 20.2).field("co", 9)]

with InfluxDBClient3(host=host,
                     token=token,
                     database=database,
                     org="Acme") as client:
    client.write(points, write_precision='s')