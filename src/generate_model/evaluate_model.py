#!/usr/bin/env python3
import argparse

import numpy as np

from keras.models import load_model

def main():
    # parse arguments
    arg_parser = argparse.ArgumentParser(description='')
    arg_parser.add_argument('MODEL_FILE', help='')
    arg_parser.add_argument('TEST_DATA', help='')
    arg_parser.add_argument('OUTPUT_FILE', help='')
    script_args = arg_parser.parse_args()

    model = load_model(script_args.MODEL_FILE)
    array_test = np.load(script_args.TEST_DATA)
    array_pred = model.predict(array_test)

    with open(script_args.OUTPUT_FILE, 'wb+') as file_out:
        file_out.write(array_pred.tobytes())

if __name__ == '__main__':
    main()
