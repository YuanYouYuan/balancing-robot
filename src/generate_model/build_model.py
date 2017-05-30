#!/usr/bin/env python3
import os
import csv
import argparse

import cv2
import numpy as np

import numpy
from keras.models import load_model
from keras.models import Sequential
from keras.layers import Dense, Dropout
from keras.optimizers import Adam

def create_model(topology):
    assert len(topology) >= 2

    def expand_string(c):
        if c == 'l':
            return 'linear'
        elif c == 's':
            return 'sigmoid'
        else:
            assert c == 'r'
            return 'relu'

    model = Sequential()
    num_input = topology[0]
    size, act = topology[1]
    model.add(Dense(size, activation=expand_string(act), input_shape=(num_input,)))

    for size, act in topology[2:]:
        assert size > 0
        if act == 'l':
            model.add(Dense(size, activation='linear'))
        elif act == 's':
            model.add(Dense(size, activation='sigmoid'))
        else:
            assert act == 'r'
            model.add(Dense(size, activation='relu'))

    opt = Adam(lr=0.001, decay=1e-5)
    model.compile(loss='binary_crossentropy', optimizer=opt)

    return model

def main():
    # parse arguments
    arg_parser = argparse.ArgumentParser(description='')
    arg_parser.add_argument('TOPOLOGY', help='for example, "2 3s 8r 1l" will create a model with 2 inputs, consisting of three layers of size 3, 8, 1 with sigmoid, relu, linear activations')
    arg_parser.add_argument('OUTPUT_DIR', help='')
    script_args = arg_parser.parse_args()

    output_dir = script_args.OUTPUT_DIR
    topology_desc = script_args.TOPOLOGY

    # create output directory if necessary
    if os.path.exists(output_dir):
        if not os.path.isdir(output_dir):
            sys.exit('%s does not exist' % output_dir)
    else:
        os.makedirs(output_dir)

    tokens = topology_desc.split()
    topology =  [int(tokens[0])] + [(int(tok[:-1]), tok[-1]) for tok in tokens[1:]]
    model = create_model(topology)

    # write files
    path_source = os.path.join(output_dir, 'example.ino')
    path_model = os.path.join(output_dir, 'model.h5')

    with open(path_source, 'w+') as file_source:
        c_source  = '''#include "model.hpp"

int input_size = %d;
int num_layer = %d;
int topo[] = {%s};
char act[] = {%s};
Model m(input_size, num_layer, topo, act);

void setup() {
    char *weight_buf = /* something generated from create_weight_buffer() */;
    m.load_weights(weight_buf);
}

void loop() {
    float *input = /* xxx */;
    float *output = /* xxx */;
    m.predict(output, input);
}
''' % (topology[0], len(topology) - 1, ', '.join(str(size) for size, _ in topology[1:]), ', '.join("'%s'" % act for _, act in topology[1:]))
        file_source.write(c_source)

    model.save(path_model)

if __name__ == '__main__':
    main()
