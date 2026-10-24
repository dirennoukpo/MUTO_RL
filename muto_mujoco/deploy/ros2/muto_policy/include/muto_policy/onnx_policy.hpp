// ONNX policy: obs (1, 29) -> action (1, 18), plus the "muto_interface" metadata stored in the model.
#pragma once

#include <array>
#include <string>

#include <onnxruntime_cxx_api.h>

#include "muto_policy/policy_io.hpp"

namespace muto_policy {

class OnnxPolicy {
public:
  explicit OnnxPolicy(const std::string & path)
  : env_(ORT_LOGGING_LEVEL_WARNING, "muto_policy"), session_(nullptr) {
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(1);  // a 29-256-128-64-18 network runs in well under 1 ms
    session_ = Ort::Session(env_, path.c_str(), opts);
    Ort::AllocatorWithDefaultOptions alloc;
    auto meta = session_.GetModelMetadata();
    auto value = meta.LookupCustomMetadataMapAllocated("muto_interface", alloc);
    if (!value) throw std::runtime_error("ONNX model has no 'muto_interface' metadata: export it with export_onnx.py");
    interface_json = value.get();
  }

  std::array<float, kAct> operator()(std::array<float, kObs> obs) {
    const std::array<int64_t, 2> shape{1, kObs};
    auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input = Ort::Value::CreateTensor<float>(mem, obs.data(), obs.size(), shape.data(), shape.size());
    const char * in_name = "obs";
    const char * out_name = "action";
    auto out = session_.Run(Ort::RunOptions{nullptr}, &in_name, &input, 1, &out_name, 1);
    const float * a = out.front().GetTensorData<float>();
    std::array<float, kAct> action{};
    std::copy(a, a + kAct, action.begin());
    return action;
  }

  std::string interface_json;

private:
  Ort::Env env_;
  Ort::Session session_;
};

}  // namespace muto_policy
