#include <ctrnn.h>
#include <vector>
#include <math.h>
#include <assert.h>

CTRNN::CTRNN(std::vector<std::vector<int>> connections,
        std::vector<double> wts,
        std::vector<double> taus,
        std::vector<double> biases,
        std::vector<double> gains,
        double step_size)
{
    ctrnnSize = connections.size();
    assert
    (
        (connections.at(0).size() == ctrnnSize) &&
        (taus.size() == ctrnnSize) &&
        (biases.size() == ctrnnSize) &&
        (gains.size() == ctrnnSize)
    );

    expanded_wts = map_wts_to_connections(wts, connections);
    this->taus = taus;
    this->gains = gains;
    this->biases = biases;
    this->step_size = step_size;

    outputs = std::vector<double>(ctrnnSize, 0.01);
    states =
        vector_vector_operation
        (
            vector_vector_operation
            (
                inverse_sigmoid(outputs),
                gains,
                '/'
            ),
            biases,
            '-'
        );
}

CTRNN::~CTRNN(void) {

}

void CTRNN::step(std::vector<double> inputs)
{
    size_t input_size = inputs.size();
    assert(input_size <= ctrnnSize);

    while (input_size < ctrnnSize)
    {
        inputs.push_back(0.0);
        input_size++;
    }

    std::vector<double> total_inputs =
        vector_vector_operation
        (
            inputs,
            wts_outputs_dot_product
            (
                expanded_wts,
                outputs
            ),
            '+'
        );

    std::vector<double> new_states =
        vector_vector_operation
        (
            states,
            vector_vector_operation
            (
                scalar_vector_operation
                (
                    step_size,
                    scalar_vector_operation
                    (
                        1.0,
                        taus,
                        '/'
                    ),
                    '*'
                ),
                vector_vector_operation
                (
                    total_inputs,
                    states,
                    '-'
                ),
                '*'
            ),
            '+'
        );

    states = new_states;

    outputs =
        sigmoid
        (
            vector_vector_operation
            (
                gains,
                vector_vector_operation
                (
                    states,
                    biases,
                    '+'
                ),
                '*'
            )
        );
}

std::vector<double> CTRNN::get_outputs(void) const
{
    return outputs;
}

// ----------------------- Private methods ---------------------------

std::vector<std::vector<double>> CTRNN::map_wts_to_connections(
    const std::vector<double>& wts, const std::vector<std::vector<int>>& connections)
{
    size_t wts_index = 0;
    std::vector<std::vector<double>> expanded_wts;

    for (size_t i = 0; i < connections.size(); i++)
    {
        std::vector<double> temp;
        for (size_t j = 0; j < connections.at(0).size(); j++)
        {
            if (connections[i][j] == 0)
            {
                temp.push_back(0.0);
            } else
            {
                temp.push_back(wts[wts_index]);
                wts_index++;
            }
        }
        expanded_wts.push_back(temp);
    }

    assert(wts_index == wts.size());
    return expanded_wts;
}

std::vector<double> CTRNN::wts_outputs_dot_product(
     const std::vector<std::vector<double>>& wts, const std::vector<double>& outputs)
{
    assert (wts.at(0).size() == outputs.size());

    std::vector<double> dot_product;

    for (size_t i = 0; i < wts.size(); i++)
    {
        double sum = 0;
        for (size_t j = 0; j < outputs.size(); j++)
        {
            sum += wts[i][j]*outputs[j];
        }
        dot_product.push_back(sum);
    }

    return dot_product;
}

std::vector<double> CTRNN::scalar_vector_operation(
    double scalar, const std::vector<double>& vector, char oper)
{
    std::vector<double> result;

    for (size_t i = 0; i < vector.size(); i++)
    {
        switch (oper) {
            case '+':
                result.push_back(scalar+vector[i]);
                break;
            case '-':
                result.push_back(scalar-vector[i]);
                break;
            case '*':
                result.push_back(scalar*vector[i]);
                break;
            case '/':
                result.push_back(scalar/vector[i]);
                break;
            default:
                assert(0);
        }
    }

    return result;
}

std::vector<double> CTRNN::vector_vector_operation(
    const std::vector<double>& vec1, const std::vector<double>& vec2, char oper)
{
    assert (vec1.size() == vec2.size());
    std::vector<double> result;

    for (size_t i = 0; i < vec1.size(); i++)
    {
        switch (oper) {
            case '+':
                result.push_back(vec1[i]+vec2[i]);
                break;
            case '-':
                result.push_back(vec1[i]-vec2[i]);
                break;
            case '*':
                result.push_back(vec1[i]*vec2[i]);
                break;
            case '/':
                result.push_back(vec1[i]/vec2[i]);
                break;
            default:
                assert(0);
        }
    }

    return result;
}

std::vector<double> CTRNN::sigmoid(const std::vector<double>& vector)
{
    std::vector<double> result;

    for (size_t i = 0; i < vector.size(); i++)
    {
        result.push_back(1 / (1+exp(-vector[i])));
    }

    return result;
}

std::vector<double> CTRNN::inverse_sigmoid(const std::vector<double>& vector)
{
    std::vector<double> result;

    for (size_t i = 0; i < vector.size(); i++)
    {
        result.push_back(log(vector[i] / (1-vector[i])));
    }

    return result;
}

// std::vector<std::vector<double>> CTRNN::connections_wts_cross_product(
//     const std::vector<std::vector<int>>& connections, const std::vector<std::vector<double>>& wts)
// {
//     assert ((connections.size() == wts.size()) &&
//         (connections.at(0).size() == wts.at(0).size()));
//
//     size_t size = connections.size();
//     std::vector<std::vector<double>> cross_product;
//
//     for (size_t i = 0; i < size; i++)
//     {
//         std::vector<double> temp;
//         for (size_t j = 0; j < size; j++)
//         {
//             temp.push_back(connections[i][j]*wts[i][j]);
//         }
//         cross_product.push_back(temp);
//     }
//
//     return cross_product;
// }
