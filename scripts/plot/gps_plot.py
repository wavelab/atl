#!/usr/bin/env python
import csv

import gmplot


gps_file = "../atl_data/170223-gps_log_test_1.csv"
# gps_file = "../atl_data/170223-gps_log_test_2.csv"

def parse_gps_data(data_file):
    # setup
    f = open(data_file, "r")
    reader = csv.reader(f, delimiter=",")
    next(reader)  # skip header

    data = {
        "t": [], "us": [],
        "gps_long": [], "gps_lat": [],
        "gps_height_ellipsoid": [],
        "gps_height_mean_sea_level": [],
        "gps_horizontal_accuracy_estimate": [],
        "gps_vertical_accuracy_estimate": []
    }

    # parse data
    for row in reader:
        data["t"].append(float(row[0]))
        data["us"].append(float(row[1]))
        data["gps_long"].append(float(row[2]))
        data["gps_lat"].append(float(row[3]))
        data["gps_height_ellipsoid"].append(float(row[4]))
        data["gps_height_mean_sea_level"].append(float(row[5]))
        data["gps_horizontal_accuracy_estimate"].append(float(row[6]))
        data["gps_vertical_accuracy_estimate"].append(float(row[7]))

    return data


if __name__ == "__main__":
    data = parse_gps_data(gps_file)

    gmap = gmplot.GoogleMapPlotter(data["gps_lat"][0], data["gps_long"][0], 16)
    gmap.plot(data["gps_lat"], data["gps_long"])
    gmap.draw("test.html")
