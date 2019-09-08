#!/bin/python
"""
Utility for pushing files to android phone using adb

python json_push

for help, run
python json_push -h
"""
import sys
import os
import argparse

ap = argparse.ArgumentParser(description='Push all json files to the phone')

ap.add_argument('-i', '--ip', default='192.168.49.1') #assume using set D
ap.add_argument('-s', '--source', default='JSON_files')
ap.add_argument('-t', '--target_dir', default='json19')
ap.add_argument('-f', '--files', nargs='*', default='*')
ap.add_argument('-a', '--adb_path', default='')
ap.add_argument('-g', '--github_path')

parsed_args = dict(vars(ap.parse_args()))

github_path = 'C:\\Users\\caden\\Documents\\Projects\\Github\\RoverRuckus\\' if parsed_args['github_path'] is None else parsed_args['github_path']
repo_path = 'TeamCode\\src\\main\\java\org\\firstinspires\\ftc\\teamcode\\' + parsed_args['source'] + '\\'
source = github_path + repo_path
adb_path = parsed_args['adb_path']
target = '/sdcard/FIRST/team9773/' + parsed_args['target_dir']

files = parsed_args['files']

if files == '*':
    os.system("{adb_path}adb.exe push * {}".format(adb_path, target))
elif len(files) != 0:
    for file in files:
        os.system("{adb_path}adb.exe push {} {}".format(adb_path, file, target))
else:
    raise Exception("Invalid files")

print("Done")
