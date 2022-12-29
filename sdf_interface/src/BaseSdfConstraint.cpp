//
// Created by qiayuan on 22-12-27.
//

#include "sdf_interface/BaseSdfConstraint.h"

namespace legged {
BaseSdfConstraint::BaseSdfConstraint(std::shared_ptr<Sdf> sdfPtr, scalar_t radius, CentroidalModelInfo info)
    : StateConstraint(ConstraintOrder::Linear), sdfPtr_(std::move(sdfPtr)), radius_(radius), info_(std::move(info)) {}

vector_t BaseSdfConstraint::getValue(scalar_t /*time*/, const vector_t& state, const PreComputation& /*preComp*/) const {
  return (vector_t(1) << sdfPtr_->value(getBasePos(state)) - radius_).finished();
}

VectorFunctionLinearApproximation BaseSdfConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                            const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(1, info_.stateDim, 0);
  approx.f = getValue(time, state, preComp);
  approx.dfdx.block(0, 0, 3, 1) = sdfPtr_->derivative(getBasePos(state));
  return approx;
}

grid_map::Position3 BaseSdfConstraint::getBasePos(const vector_t& state) const {
  return grid_map::Position3(centroidal_model::getBasePose(state, info_).head(3));
}

}  // namespace legged
