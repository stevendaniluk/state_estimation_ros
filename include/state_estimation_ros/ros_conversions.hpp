
namespace state_estimation {
template <uint32_t N>
void covarianceMatrixToVector(const Eigen::MatrixXd& cov_in, const std::array<int, N>& map,
                              boost::array<double, N * N>* cov_out) {
    // Go through each element in the covariance matrix checking if it is defined in the map and
    // copying the covariance over when so
    for (uint32_t row = 0; row < N; ++row) {
        if (map[row] >= 0) {
            for (uint32_t col = 0; col < N; ++col) {
                if (map[col] >= 0) {
                    uint32_t index = row * N + col;
                    cov_out->at(index) = cov_in(map[row], map[col]);
                }
            }
        }
    }
}

template <uint32_t N>
void covarianceVectorToMatrix(const boost::array<double, N * N>& cov_in,
                              const std::array<int, N>& map, Eigen::MatrixXd* cov_out) {
    // Go through each element in the covariance vector checking if it is defined in the map and
    // copying the covariance over when so
    for (uint32_t row = 0; row < N; ++row) {
        if (map[row] >= 0) {
            for (uint32_t col = 0; col < N; ++col) {
                if (map[col] >= 0) {
                    uint32_t index = row * N + col;
                    (*cov_out)(map[row], map[col]) = cov_in[index];
                }
            }
        }
    }
}

}  // namespace state_estimation
