#ifndef _CTRNN_H
#define _CTRNN_H

#include <vector>

class CTRNN
{
    private:
      size_t ctrnnSize;
    	std::vector<std::vector<double>> expanded_wts;
    	std::vector<double> taus;
    	std::vector<double> biases;
    	std::vector<double> gains;
    	double step_size;

    	std::vector<double> states;
    	std::vector<double> outputs;

    public:
        CTRNN (std::vector<std::vector<int>>,
               std::vector<double>,
               std::vector<double>,
               std::vector<double>,
               std::vector<double>,
               double);

        ~CTRNN(void);

        void step(std::vector<double>);

        std::vector<double> get_outputs(void) const;

    private:
        std::vector<std::vector<double>> map_wts_to_connections(
            const std::vector<double>&, const std::vector<std::vector<int>>&);

        std::vector<double> wts_outputs_dot_product(
	        const std::vector<std::vector<double>>&, const std::vector<double>&);

		std::vector<double> scalar_vector_operation(
		    double, const std::vector<double>&, char);

		std::vector<double> vector_vector_operation(
			const std::vector<double>&, const std::vector<double>&, char);

		std::vector<double> sigmoid(const std::vector<double>&);

		std::vector<double> inverse_sigmoid(const std::vector<double>&);

	// std::vector<std::vector<int>> connections;
    // connections.resize(2, std::vector<int>(2,1));
	//
    // std::vector<std::vector<double>> wts{{4.5, 1},
    //                             {-1, 4.5}};
	//
    // std::vector<double> taus = std::vector<double>(2,1.0);
    // std::vector<double> biases = std::vector<double>{-2.75,-1.75};
    // std::vector<double> gains = std::vector<double>(2,1.0);
	//
    // CTRNN ctrnn(connections, wts, taus, biases, gains, 0.01);
	//
    // std::vector<std::vector<double>> result;
    // for (int i = 0; i < 1000; i++)
    // {
    //     std::vector<double> inputs{1,1};
    //     ctrnn.step(inputs);
    //     std::vector<double> outputs = ctrnn.get_outputs();
    //     result.push_back(outputs);
    // }
	//
    // for (size_t i = 0; i < result.size(); i++) {
    //     for (size_t j = 0; j < result.at(0).size(); j++) {
    //         std::cout << result[i][j] << ' ';
    //     }
    //     std::cout << std::endl;
    // }
};

#endif
