#include "theory/bv/bv_solver_equality.h"

#include <cstddef>
#include <unordered_map>
#include <vector>

#include "expr/type_node.h"
#include "theory/bv/theory_bv_utils.h"
#include "theory/theory_inference_manager.h"
#include "theory/uf/equality_engine_iterator.h"
#include "util/bitvector.h"

namespace cvc5::internal {
namespace theory {
namespace bv {

BVSolverEquality::BVSolverEquality(Env& env,
                                   TheoryState& state,
                                   TheoryInferenceManager& inferMgr)
    : BVSolver(env, state, inferMgr)
{
}

void BVSolverEquality::setEqualityEngine(eq::EqualityEngine* ee) { d_ee = ee; }

void BVSolverEquality::postCheck(Theory::Effort level)
{
  // Find out if there are too many BV equivalence classes for a given BV size.
  uint32_t bvSize;
  std::unordered_map<uint32_t, std::vector<Node>> bvSizeRepresentatives;
  eq::EqClassesIterator it(d_ee);
  bool overflow = false;
  do
  {
    auto&& representative = *it;
    auto&& type = representative.getType();
    if (!type.isBitVector())
    {
      continue;
    }
    bvSize = type.getBitVectorSize();
    bvSizeRepresentatives[bvSize].push_back(representative);
    if (bvSizeRepresentatives[bvSize].size() == 2ull << bvSize)
    {
      overflow = true;
    }
  } while (!(++it).isFinished() && !overflow);

  if (!overflow)
  {
    // There are no equality classes without overflow, nothing to do.
    return;
  }

  // Add lemma stating that some two of the equality class reps are equal.
  // The two equality classes represented by them should be merged.
  std::vector<Node> equalities;
  auto& representatives = bvSizeRepresentatives[bvSize];
  for (std::size_t i = 0; i < representatives.size(); i++)
  {
    auto& r_i = representatives[i];
    for (std::size_t j = i + 1; j < representatives.size(); j++)
    {
      auto& r_j = representatives[j];
      equalities.push_back(r_i.eqNode(r_j));
    }
  }
  d_im.lemma(utils::mkOr(equalities), InferenceId::BV_EQ_DISJUNCTION_LEMMA);
}

TrustNode BVSolverEquality::explain(TNode n) { return TrustNode::null(); }

bool BVSolverEquality::collectModelValues(TheoryModel* m,
                                          const std::set<Node>& termSet)
{
  // Model is sound, as far as we know.
  return true;
}

}  // namespace bv
}  // namespace theory
}  // namespace cvc5::internal
