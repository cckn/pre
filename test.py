# -*- coding: UTF-8 -*-

import threading

import ReportGPS
import ReportDistance


class report_gps(threading.Thread):

    def __init__(self, config_path):
        super(report_gps, self).__init__()
        self.config_path = config_path

    def run(self):
        rg = ReportGPS.ReportGPS(self.config_path)
        rg.run()


class report_distance(threading.Thread):

    def __init__(self, config_path):
        super(report_distance, self).__init__()
        self.config_path = config_path

    def run(self):
        rg = ReportDistance.ReportDistance(self.config_path)
        rg.run()

if __name__ == '__main__':

    thread1 = report_gps("config.conf")
    thread2 = report_distance("config.conf")
    thread1.demon = True
    thread2.demon = True
    thread1.start()
    thread2.start()
