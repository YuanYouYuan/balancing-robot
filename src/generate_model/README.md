## build model
To create a model with 2 inputs, consisting of three layers of size 3, 8, 1 with sigmoid, relu, linear activations,
``
python ./build\_model.py '2 3r 8s 1l' out_dir
``

This command also generates an example Arduino source `example.ino`.

## serail model weights
This will create a `out_dir` directory, containing file model dump file `model.h5`.

`create_weight_buffer.py` is the example script to serialize the weight of a model into binary array.

You may use `create_weight_buffer(model)` function freely.

## example usage

1. Run `build_model.py` to create a model. Load `model.h5` on Python side. Include C sources in `src` in your Arduino project.
2. Train your model on Python side, and then serialize the model weights using `create_weight_buffer(model)`.
3. Pass your serialized weights to the Arduino side.
