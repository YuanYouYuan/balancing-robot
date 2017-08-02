class Model
{
private:
    int input_size;
    int max_layer_size;
    int num_layer;
    int weight_buffer_size;
    int *topo;
    char *act;
    float *weight_buffer;
    float *bias_buffer;
    void mat_mul(float *dst, const float *src, float *mat, int rows, int cols);
    void vector_add(float *dst, const float *src, int layer_size);
    void vector_sigmoid(float *dst, int layer_size);
    void vector_relu(float *dst, int layer_size);

public:
    Model(const int input_size, int num_layer, const int _topo[], const char _act[]);
    ~Model();
    void load_weights(const float *_weight_buffer);
    void predict(float output[], const float input[]);
};
