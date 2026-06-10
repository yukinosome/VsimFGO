//  Lightweight feed-forward network for GP residual interpolation.
//
//  This header intentionally has no runtime dependency on Torch or ONNX. It is
//  meant for the first engineering pass where the network is a small exported
//  MLP and its output is treated as a fixed residual during factor
//  linearization.

#ifndef ONLINE_FGO_SIMPLEMLP_H
#define ONLINE_FGO_SIMPLEMLP_H

#pragma once

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

namespace fgo::models::nn {

  class SimpleMLP {
  public:
    enum class Activation {
      Linear,
      Relu,
      Tanh,
      Silu
    };

    struct Layer {
      Eigen::MatrixXd weight;
      Eigen::VectorXd bias;
      Activation activation{Activation::Linear};
    };

    SimpleMLP() = default;

    explicit SimpleMLP(std::vector<Layer> layers) : layers_(std::move(layers)) {
      validate();
    }

    [[nodiscard]] bool empty() const {
      return layers_.empty();
    }

    [[nodiscard]] int inputDim() const {
      return layers_.empty() ? 0 : static_cast<int>(layers_.front().weight.cols());
    }

    [[nodiscard]] int outputDim() const {
      return layers_.empty() ? 0 : static_cast<int>(layers_.back().weight.rows());
    }

    [[nodiscard]] const std::vector<Layer> &layers() const {
      return layers_;
    }

    [[nodiscard]] Eigen::VectorXd forward(const Eigen::VectorXd &input) const {
      if (layers_.empty()) {
        return Eigen::VectorXd();
      }
      if (input.size() != layers_.front().weight.cols()) {
        throw std::runtime_error("SimpleMLP input dimension mismatch");
      }

      Eigen::VectorXd x = input;
      for (const auto &layer: layers_) {
        x = layer.weight * x + layer.bias;
        applyActivationInPlace(x, layer.activation);
      }
      return x;
    }

    /**
     * Load a text-exported MLP.
     *
     * Format:
     *   simple_mlp_v1
     *   <num_layers>
     *   <out_dim> <in_dim> <activation>
     *   <out_dim * in_dim row-major weights>
     *   <out_dim biases>
     *   ... repeated for each layer
     *
     * Activation names: linear, relu, tanh, silu.
     */
    void loadFromTextFile(const std::string &path) {
      std::ifstream stream(path);
      if (!stream.is_open()) {
        throw std::runtime_error("Cannot open SimpleMLP weight file: " + path);
      }

      std::string magic;
      stream >> magic;
      if (magic != "simple_mlp_v1") {
        throw std::runtime_error("Unsupported SimpleMLP weight file format: " + magic);
      }

      size_t num_layers = 0;
      stream >> num_layers;
      if (!stream || num_layers == 0) {
        throw std::runtime_error("Invalid SimpleMLP layer count");
      }

      std::vector<Layer> loaded_layers;
      loaded_layers.reserve(num_layers);
      for (size_t layer_index = 0; layer_index < num_layers; ++layer_index) {
        int out_dim = 0;
        int in_dim = 0;
        std::string activation_name;
        stream >> out_dim >> in_dim >> activation_name;
        if (!stream || out_dim <= 0 || in_dim <= 0) {
          throw std::runtime_error("Invalid SimpleMLP layer header");
        }

        Layer layer;
        layer.weight.resize(out_dim, in_dim);
        layer.bias.resize(out_dim);
        layer.activation = parseActivation(activation_name);

        for (int row = 0; row < out_dim; ++row) {
          for (int col = 0; col < in_dim; ++col) {
            stream >> layer.weight(row, col);
          }
        }

        for (int row = 0; row < out_dim; ++row) {
          stream >> layer.bias(row);
        }

        if (!stream) {
          throw std::runtime_error("Unexpected end of SimpleMLP weight file");
        }

        loaded_layers.push_back(std::move(layer));
      }

      layers_ = std::move(loaded_layers);
      validate();
    }

    static Activation parseActivation(const std::string &name) {
      if (name == "linear") {
        return Activation::Linear;
      }
      if (name == "relu") {
        return Activation::Relu;
      }
      if (name == "tanh") {
        return Activation::Tanh;
      }
      if (name == "silu") {
        return Activation::Silu;
      }
      throw std::runtime_error("Unknown SimpleMLP activation: " + name);
    }

  private:
    std::vector<Layer> layers_;

    void validate() const {
      if (layers_.empty()) {
        return;
      }
      for (size_t i = 0; i < layers_.size(); ++i) {
        const auto &layer = layers_[i];
        if (layer.weight.rows() != layer.bias.size()) {
          throw std::runtime_error("SimpleMLP layer weight/bias dimension mismatch");
        }
        if (i > 0 && layer.weight.cols() != layers_[i - 1].weight.rows()) {
          throw std::runtime_error("SimpleMLP adjacent layer dimension mismatch");
        }
      }
    }

    static void applyActivationInPlace(Eigen::VectorXd &x, Activation activation) {
      switch (activation) {
        case Activation::Linear:
          return;
        case Activation::Relu:
          x = x.cwiseMax(0.0);
          return;
        case Activation::Tanh:
          x = x.array().tanh().matrix();
          return;
        case Activation::Silu:
          x = (x.array() / (1.0 + (-x.array()).exp())).matrix();
          return;
      }
    }
  };

} // namespace fgo::models::nn

#endif // ONLINE_FGO_SIMPLEMLP_H
