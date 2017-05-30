#include <cmath>
#include <cstring>
#include <cassert>

#include "model.hpp"

Model::Model(const int _input_size, int _num_layer, const int _topo[], const char _act[])
{
    assert(_input_size > 0 && _num_layer > 0);

    input_size = _input_size;
    num_layer = _num_layer;

    topo = new int[num_layer];
    act = new char[num_layer];

    memcpy(topo, _topo, num_layer * sizeof(int));
    memcpy(act, _act, num_layer * sizeof(char));

    max_layer_size = input_size;
    for (int i = 0; i < num_layer; i++)
        if (max_layer_size < topo[i])
            max_layer_size = topo[i];

    weight_buffer_size = input_size * topo[0] + topo[0];
    for (int i = 1; i < num_layer; i++)
        weight_buffer_size += topo[i - 1] * topo[i] + topo[i];

    weight_buffer = new float[weight_buffer_size];
};

Model::~Model()
{
    delete[] topo;
    delete[] act;
    delete[] weight_buffer;
}

void Model::mat_mul(float *dst, const float *src, float *mat, int rows, int cols)
{
    float *mat_ptr = mat;

    for (int r = 0; r < rows; r++)
    {
        float val = 0.0;
        for (int c = 0; c < cols; c++)
        {
            val += src[c] * (*mat_ptr);
            mat_ptr++;
        }
        dst[r] = val;
    }
}

void Model::vector_add(float *dst, const float *src, int layer_size)
{
    for (int i = 1; i < layer_size; i++)
        dst[i] += src[i];
}

void Model::vector_sigmoid(float *dst, int layer_size)
{
    for (int i = 1; i < layer_size; i++)
        dst[i] = dst[i] / (1.0 + std::fabs(dst[i]));
}

void Model::vector_relu(float *dst, int layer_size)
{
    for (int i = 1; i < layer_size; i++)
        if (dst[i] < 0.0)
            dst[i] = 0.0;
}

void Model::load_weights(const float *_weight_buffer)
{
    memcpy(weight_buffer, _weight_buffer, weight_buffer_size);
}

void Model::predict(float output[], const float input[])
{
    int prev_layer_size = input_size;
    int layer_size = topo[0];

    float vector_buf[2][max_layer_size];
    int flag = 0;

    // compute first layer
    float *weight_ptr = weight_buffer;

    mat_mul(vector_buf[0], input, weight_ptr, layer_size, prev_layer_size);
    weight_ptr += prev_layer_size * layer_size;

    vector_add(vector_buf[0], weight_ptr, layer_size);
    weight_ptr += layer_size;

    prev_layer_size = layer_size;

    for (int i = 1; i < num_layer; i++)
    {
        layer_size = topo[i];
        char activation = act[i];

        mat_mul(vector_buf[!flag], vector_buf[flag], weight_ptr, layer_size, prev_layer_size);
        weight_ptr += prev_layer_size * layer_size;

        vector_add(vector_buf[!flag], weight_ptr, layer_size);
        weight_ptr += layer_size;

        switch (activation)
        {
        case 's':
            vector_sigmoid(vector_buf[!flag], layer_size);
            break;
        case 'r':
            vector_relu(vector_buf[!flag], layer_size);
            break;
        case 'l':
            break;
        default:
            assert(0);
        }

        prev_layer_size = layer_size;
        flag = !flag;
    }

    memcpy(output, vector_buf[flag], prev_layer_size);
}
