#include "theory/bv/bv_solver_subtheories.h"

#include <memory>

#include "options/bv_options.h"
#include "theory/bv/bv_solver_bitblast.h"
#include "theory/bv/bv_solver_bitblast_internal.h"
#include "theory/bv/bv_solver_equality.h"
#include "theory/bv/bv_solver_inequality.h"

namespace cvc5::internal {
namespace theory {
namespace bv {

BVSolverSubtheories::BVSolverSubtheories(Env& env,
                                         TheoryState& state,
                                         TheoryInferenceManager& inferMgr)
    : BVSolver(env, state, inferMgr)
{
  switch (options().bv.bvSolver)
  {
    case options::BVSolver::BITBLAST:
      d_bitBlastSolver.reset(new BVSolverBitblast(d_env, &d_state, d_im));
      break;

    default:
      AlwaysAssert(options().bv.bvSolver
                   == options::BVSolver::BITBLAST_INTERNAL);
      d_bitBlastSolver.reset(
          new BVSolverBitblastInternal(d_env, &d_state, d_im));
  }

  if (options().bv.bitblastMode != options::BitblastMode::EAGER)
  {
    // Optionally add subtheories.
    if (options().bv.bvEqualitySolver)
    {
      d_subSolvers.push_back(
          std::make_unique<BVSolverEquality>(d_env, d_state, d_im));
    }
    if (options().bv.bvInEqualitySolver)
    {
      d_subSolvers.push_back(
          std::make_unique<BVSolverInEquality>(d_env, d_state, d_im));
    }
  }
}

bool BVSolverSubtheories::needsEqualityEngine(EeSetupInfo& esi)
{
  return d_bitBlastSolver->needsEqualityEngine(esi);
}

void BVSolverSubtheories::setEqualityEngine(eq::EqualityEngine* ee)
{
  for (auto&& subSolver : d_subSolvers)
  {
    subSolver->setEqualityEngine(ee);
  }
  d_bitBlastSolver->setEqualityEngine(ee);
}

void BVSolverSubtheories::postCheck(Theory::Effort level)
{
  for (auto&& subSolver : d_subSolvers)
  {
    subSolver->postCheck(level);
  }
  d_bitBlastSolver->postCheck(level);
}

bool BVSolverSubtheories::preNotifyFact(
    TNode atom, bool pol, TNode fact, bool isPrereg, bool isInternal)
{
  for (auto&& subSolver : d_subSolvers)
  {
    subSolver->preNotifyFact(atom, pol, fact, isPrereg, isInternal);
  }
  // Let bit-blasting solver decide if fact should be asserted to equality
  // engine.
  return d_bitBlastSolver->preNotifyFact(atom, pol, fact, isPrereg, isInternal);
}

void BVSolverSubtheories::computeRelevantTerms(std::set<Node>& termSet)
{
  for (auto&& subSolver : d_subSolvers)
  {
    subSolver->computeRelevantTerms(termSet);
  }
  d_bitBlastSolver->computeRelevantTerms(termSet);
}

bool BVSolverSubtheories::collectModelValues(TheoryModel* m,
                                             const std::set<Node>& termSet)
{
  return d_bitBlastSolver->collectModelValues(m, termSet);
}

TrustNode BVSolverSubtheories::explain(TNode n)
{
  // Subsolvers that didn't propagate a node will return a null (invalid) node.
  for (auto&& subSolver : d_subSolvers)
  {
    auto&& node = subSolver->explain(n);
    if (node.getKind() != TrustNodeKind::INVALID)
    {
      return node;
    }
  }
  return d_bitBlastSolver->explain(n);
}

Node BVSolverSubtheories::getValue(TNode node, bool initialize)
{
  return d_bitBlastSolver->getValue(node, initialize);
}

BVProofRuleChecker* BVSolverSubtheories::getProofChecker()
{
  return d_bitBlastSolver->getProofChecker();
}

}  // namespace bv
}  // namespace theory
}  // namespace cvc5::internal
