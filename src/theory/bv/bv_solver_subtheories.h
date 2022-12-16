#include <vector>

#include "cvc5_private.h"

#ifndef CVC5__THEORY__BV__BV_SOLVER_SUBTHEORIES_H
#define CVC5__THEORY__BV__BV_SOLVER_SUBTHEORIES_H

#include "theory/bv/bv_solver.h"

namespace cvc5::internal {

namespace theory {
namespace bv {

class BVSolverSubtheories : public BVSolver
{
 public:
  BVSolverSubtheories(Env& env,
                      TheoryState& state,
                      TheoryInferenceManager& inferMgr);
  ~BVSolverSubtheories() = default;

  //--------------------------------- initialization
  bool needsEqualityEngine(EeSetupInfo& esi) override;

  //--------------------------------- check
  void postCheck(Theory::Effort level = Theory::Effort::EFFORT_FULL) override;
  bool preNotifyFact(TNode atom,
                     bool pol,
                     TNode fact,
                     bool isPrereg,
                     bool isInternal) override;

  //--------------------------------- collect model info
  void computeRelevantTerms(std::set<Node>& termSet) override;
  bool collectModelValues(TheoryModel* m,
                          const std::set<Node>& termSet) override;

  TrustNode explain(TNode n) override;

  std::string identify() const override { return "BVSolverSubtheories"; };

  Node getValue(TNode node, bool initialize) override;

  BVProofRuleChecker* getProofChecker() override;

 protected:
  /* Bit-blasting solver subclass. */
  std::unique_ptr<BVSolver> d_bitBlastSolver;

  /* Ordered list of (algebraic) subtheory solvers. */
  std::vector<std::unique_ptr<BVSolver>> d_subSolvers;
};

}  // namespace bv
}  // namespace theory
}  // namespace cvc5::internal

#endif /* CVC5__THEORY__BV__BV_SOLVER_SUBTHEORIES_H */
