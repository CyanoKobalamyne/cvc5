#include "theory/bv/bv_solver_subtheories.h"

#include <memory>

#include "options/bv_options.h"
#include "theory/bv/bv_solver_bitblast.h"
#include "theory/bv/bv_solver_bitblast_internal.h"

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
      d_bitBlastSolver.reset(new BVSolverBitblast(env, &state, inferMgr));
      break;

    default:
      AlwaysAssert(options().bv.bvSolver
                   == options::BVSolver::BITBLAST_INTERNAL);
      d_bitBlastSolver.reset(
          new BVSolverBitblastInternal(env, &state, inferMgr));
  }
}

bool BVSolverSubtheories::needsEqualityEngine(EeSetupInfo& esi)
{
  return d_bitBlastSolver->needsEqualityEngine(esi);
}

void BVSolverSubtheories::postCheck(Theory::Effort level)
{
  d_bitBlastSolver->postCheck(level);
}

bool BVSolverSubtheories::preNotifyFact(
    TNode atom, bool pol, TNode fact, bool isPrereg, bool isInternal)
{
  return d_bitBlastSolver->preNotifyFact(atom, pol, fact, isPrereg, isInternal);
}

void BVSolverSubtheories::computeRelevantTerms(std::set<Node>& termSet)
{
  d_bitBlastSolver->computeRelevantTerms(termSet);
}

bool BVSolverSubtheories::collectModelValues(TheoryModel* m,
                                             const std::set<Node>& termSet)
{
  return d_bitBlastSolver->collectModelValues(m, termSet);
}

TrustNode BVSolverSubtheories::explain(TNode n)
{
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
