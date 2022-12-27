//
// Created by qiayuan on 22-12-27.
//

#pragma once

#include <memory>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <grid_map_sdf/SignedDistanceField.hpp>

namespace legged {

using namespace ocs2;

class BaseSdfConstraint final : public ocs2::StateConstraint {
 public:
  BaseSdfConstraint(std::shared_ptr<grid_map::SignedDistanceField> sdfPtr, scalar_t radius, CentroidalModelInfo info);

  /** Default destructor */
  ~BaseSdfConstraint() override = default;

  BaseSdfConstraint* clone() const override { return new BaseSdfConstraint(*this); }
  size_t getNumConstraints(scalar_t /*time*/) const override { return 1; }

  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComp) const override;

 private:
  grid_map::Position3 getBasePos(const vector_t& state) const;

  BaseSdfConstraint(const BaseSdfConstraint& other) = default;

  std::shared_ptr<grid_map::SignedDistanceField> sdfPtr_;
  scalar_t radius_;
  CentroidalModelInfo info_;
};

}  // namespace legged