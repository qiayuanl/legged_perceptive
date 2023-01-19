//
// Created by qiayuan on 22-12-27.
//

#pragma once

#include "legged_perceptive_interface/constraint/SphereSdfConstraint.h"

#include <convex_plane_decomposition/PlanarRegion.h>
#include <legged_interface/LeggedInterface.h>

namespace legged {

class SphereSdfLeggedInterface : public legged::LeggedInterface {
 public:
  using LeggedInterface::LeggedInterface;

  void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                  bool verbose) override;

  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> getPlanarTerrainPtr() const { return planarTerrainPtr_; }
  std::shared_ptr<grid_map::SignedDistanceField> getSdfPrt() const { return sdfPrt_; }
  std::shared_ptr<PinocchioSphereInterface> getPinocchioSphereInterfacePrt() const { return pinocchioSphereInterfacePrt_; }

 private:
  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
  std::shared_ptr<grid_map::SignedDistanceField> sdfPrt_;
  std::shared_ptr<PinocchioSphereInterface> pinocchioSphereInterfacePrt_;
};

class FootPlacementLeggedInterface : public legged::LeggedInterface {
 public:
  using LeggedInterface::LeggedInterface;

  void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                  bool verbose) override;

  void setupReferenceManager(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                             bool verbose) override;

  void setupPreComputation(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                           bool verbose) override;

  std::shared_ptr<grid_map::SignedDistanceField> getSdfPrt() const { return sdfPrt_; }

  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> getPlanarTerrainPtr() const { return planarTerrainPtr_; }

  size_t getNumVertices() const { return numVertices_; }

 private:
  size_t numVertices_ = 16;

  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
  std::shared_ptr<grid_map::SignedDistanceField> sdfPrt_;
};

}  // namespace legged
