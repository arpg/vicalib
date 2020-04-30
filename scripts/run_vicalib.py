from __future__ import print_function
import tempfile
import subprocess
import glob
import os
import re
import argparse
import csv
import shutil
from dateutil.parser import parse
import pandas as pd
from scipy.spatial import ConvexHull
import json
import numpy as np


def main(args):


    if not args.process_existing:
      #if process_exist skip vicalib call
      temp_out_dir = tempfile.mkdtemp()

      if args.output_json is None:
        args.output_json = os.path.join(temp_out_dir, "camera.json")

      command_str = [args.vicalib_path,
                     "-grid_spacing", "0.032",         #grid spacing for large target in the tester
                     "-grid_large_rad", "0.0104",      #grid large cirle size for large target in the tester
                     "-grid_small_rad", "0.0065",      #grid small cirle size spacing for large target in the tester
                     "-pattern_preset_only", "large",  #name of the large target in the tester
                     "-output_json", args.output_json,  # Set to avoid extra output in current directory
                     "-output", os.path.join(temp_out_dir, "camera.xml"),        # Set to avoid extra output in current directory
                     "-input_json", args.input_json,
                     "-log_dir", temp_out_dir,
                     "-accel_sigma", "0.0707",
                     "-gyro_sigma", "0.01414",
                     "-calibrate_imu=false",
                     "-calibrate_intrinsics",
                     "-has_initial_guess",
                     "-imu", "csv://" + args.input_data_folder, #path to the imu data used for processing
                     "-cam", "file://" + args.input_data_folder + "/[vifisheye_0_,vifisheye_1_]*.pgm"]  #path the images used for processing
      try:
          subprocess.check_output(command_str, stderr=subprocess.STDOUT)
      except subprocess.CalledProcessError as e:
          print(e.output)
      log_file_path = os.path.join(temp_out_dir,"*INFO.*-*.*")
      image_files_path = os.path.join(args.input_data_folder,"vifisheye_0_*")
      reprojection_error_file_path = os.path.join(temp_out_dir,"reprojection_error.csv")
    else:
      log_file_path = os.path.join(args.input_data_folder,"vicalibOutput","*INFO.*-*.*")
      image_files_path = ""
      reprojection_error_file_path = os.path.join(args.input_data_folder,"vicalibOutput","reprojection_error.csv")

    log_files = glob.glob(log_file_path)

    number_of_images = len(glob.glob(image_files_path))

    if len(log_files) != 1:
        print('More than one or zero *INFO* files in %s wile processing %s' % (temp_out_dir, args.input_data_folder))
        return list(np.ones([30 + 14 + 1])*np.nan)

    result_file  = log_files[0]
    with open(result_file,"r") as f3:
        lines = f3.read()

        timestamps = re.findall("([0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]{6})", lines)

        duration = (parse(timestamps[-1]) - parse(timestamps[0])).total_seconds()

        if duration < 0:
          duration+=24*60*60 #the run ran over midnight

        fisheye0_strs = re.findall("Reprojection error for camera 0: .* rmse: (.+)", lines)
        if len(fisheye0_strs) == 1:
            fisheye0 = float(fisheye0_strs[0])
        else:
            fisheye0 = float('nan')

        fisheye1_strs = re.findall("Reprojection error for camera 1: .* rmse: (.+)", lines)
        if len(fisheye1_strs) == 1:
            fisheye1 = float(fisheye1_strs[0])
        else:
            fisheye1 = float('nan')

        imucost_strs = re.findall("Imu cost : .* rmse: (.+)", lines)
        if len(imucost_strs) == 1:
            imucost = float(imucost_strs[0])
        else:
            imucost = float('nan')

    try:
      reprojection_error = pd.read_csv(reprojection_error_file_path, header=None)
    except Exception as e:
      print(e)
      return [fisheye0, fisheye1, imucost, duration] + list(np.ones([26 + 14])*np.nan) + [number_of_images]

    calibration_json = args.output_json

    try:
      with open(calibration_json) as f:
          calibration = json.load(f)
    except FileNotFoundError as e:
      print("%s not found" % calibration_json)
      return [fisheye0, fisheye1, imucost, duration] + list(np.ones([26 + 14])*np.nan) + [number_of_images]

    radial_extent = []
    for index in range(2):
        cc = calibration["cameras"][index]["center_px"]
        points0 = np.vstack([reprojection_error[7][reprojection_error[0] == index],reprojection_error[8][reprojection_error[0] == index]]).T
        hull0 = ConvexHull(points0)
        radial_extent.append(np.max(np.linalg.norm(np.kron(cc, np.ones([len(points0[hull0.vertices,:]),1]))-points0[hull0.vertices,:],axis=1)))

    if not args.process_existing:
      shutil.rmtree(temp_out_dir)

    return [fisheye0, fisheye1, imucost, duration] + radial_extent + calibration["cameras"][0]["focal_length_px"] + calibration["cameras"][0]["distortion"]["k"] + calibration["cameras"][1]["focal_length_px"] + calibration["cameras"][1]["distortion"]["k"] + calibration["imus"][0]["accelerometer"]["bias"] + calibration["imus"][0]["accelerometer"]["scale_and_alignment"][0:9:4] + calibration["imus"][0]["gyroscope"]["bias"] + calibration["imus"][0]["gyroscope"]["scale_and_alignment"][0:9:4] + calibration['cameras'][0]['extrinsics']['T'] + calibration['cameras'][0]['extrinsics']['W'] + calibration['cameras'][1]['extrinsics']['T'] + calibration['cameras'][1]['extrinsics']['W'] + [np.abs(calibration['cameras'][0]['extrinsics']['T'][0] - calibration['cameras'][1]['extrinsics']['T'][0])] + [np.linalg.norm(np.array(calibration['cameras'][0]['extrinsics']['T']) - np.array(calibration['cameras'][1]['extrinsics']['T']))] + [number_of_images]



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='''
    Run vicalib
    ''')

    dir_path = os.path.dirname(os.path.realpath(__file__))

    parser.add_argument('input_json', help='input json file in multiple camera format', default=os.path.join(dir_path, "..", "data", "t26x-initialization.json"))
    parser.add_argument('input_data_folder', help='input data folder')
    parser.add_argument('vicalib_path', help='path to vcalib executable',  default=r"./vicalib")
    parser.add_argument('output_json', help='output json file in multiple camera format',  default=None)
    parser.add_argument('-p', '--process_existing', help='process existing results files', action='store_true')

    args = parser.parse_args()
    print(main(args))

