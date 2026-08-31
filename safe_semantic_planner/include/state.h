#pragma once
#include <cstdint>
#include <vector>
#include <cmath>

class State {
public:
    uint64_t id;
    std::vector<double> embedding;

    State() : id(0) {}
    State(uint64_t id_, std::vector<double> embedding_)
        : id(id_), embedding(std::move(embedding_)) {}

    static double euclidean(const State& a, const State& b) {
        double sum = 0.0;
        size_t d = std::min(a.embedding.size(), b.embedding.size());
        for (size_t i = 0; i < d; ++i) {
            double diff = a.embedding[i] - b.embedding[i];
            sum += diff * diff;
        }
        return std::sqrt(sum);
    }
};
