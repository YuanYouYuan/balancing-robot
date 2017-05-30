#!/usr/bin/env python3
import argparse

from keras.models import load_model

def main():
    # parse arguments
    arg_parser = argparse.ArgumentParser(description='')
    arg_parser.add_argument('SRC_MODEL', help='')
    arg_parser.add_argument('DST_MODEL', help='')
    arg_parser.add_argument('TRAIN_DATA', help='')
    arg_parser.add_argument('LABEL_DATA', help='')
    arg_parser.add_argument('EPOCHES', type=int, help='')
    script_args = arg_parser.parse_args()

    model = load_model(script_args.SRC_MODEL)
    array_train = numpy.load(script_args.TRAIN_DATA)
    array_label = numpy.load(script_args.LABEL_DATA).reshape([-1, 1])

    model.fit(array_train, array_label,
              batch_size=32,
              epochs=script_args.EPOCHES,
              shuffle=True)

    model.save(script_args.DST_MODEL)
    
if __name__ == '__main__':
    main()
