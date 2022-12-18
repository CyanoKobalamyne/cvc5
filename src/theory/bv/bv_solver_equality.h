#ifndef CVC5__THEORY__BV__BV_SOLVER_EQUALITY_H
#define CVC5__THEORY__BV__BV_SOLVER_EQUALITY_H

#include "theory/bv/bv_solver.h"
#include "theory/uf/equality_engine.h"

namespace cvc5::internal {

namespace theory {
namespace bv {

class BVSolverEquality : public BVSolver
{
 public:
  BVSolverEquality(Env& env,
                   TheoryState& state,
                   TheoryInferenceManager& inferMgr);
  ~BVSolverEquality() = default;

  void setEqualityEngine(eq::EqualityEngine* ee) override;

  void postCheck(Theory::Effort level = Theory::Effort::EFFORT_FULL) override;

  bool collectModelValues(TheoryModel* m,
                          const std::set<Node>& termSet) override;

  std::string identify() const override { return "BVSolverEquality"; };

 private:
  eq::EqualityEngine* d_ee;
};

}  // namespace bv
}  // namespace theory
}  // namespace cvc5::internal

#endif /* CVC5__THEORY__BV__BV_SOLVER_EQUALITY_H */
