#!/usr/bin/env python2
import time
import matplotlib.pylab as plt

if __name__ == "__main__":
    data_file = open("update.dat", "r")

    wp_start = []
    wp_end = []
    position = []
    carrot = []

    data = {
        "wp_start": [],
        "wp_end": [],
        "position": [],
        "carrot": []
    }

    index = 0
    for line in data_file.read().split("\n"):
        if line == "":
            data["wp_start"].append(wp_start)
            data["wp_end"].append(wp_end)
            data["position"].append(position)
            data["carrot"].append(carrot)

            wp_start = []
            wp_end = []
            position = []
            carrot = []
            index = 0

        else:
            el = line.split(",")
            for e in el:
                if index == 0:
                    wp_start.append(float(e.strip()))
                elif index == 1:
                    wp_end.append(float(e.strip()))
                elif index == 2:
                    position.append(float(e.strip()))
                elif index == 3:
                    carrot.append(float(e.strip()))
            index += 1

    data["wp_start"].pop()
    data["wp_end"].pop()
    data["position"].pop()
    data["carrot"].pop()

    plt.ion()
    for i in range(len(data["wp_start"])):
        wp_start = data["wp_start"][i]
        wp_end = data["wp_end"][i]
        position = data["position"][i]
        carrot = data["carrot"][i]

        plt.plot([wp_start[0], wp_end[0]], [wp_start[1], wp_end[1]], 'r-')
        plt.plot(carrot[0], carrot[1], 'ro')
        plt.plot(position[0], position[1], 'bo')
        plt.show()
        plt.draw()
        time.sleep(0.1)
