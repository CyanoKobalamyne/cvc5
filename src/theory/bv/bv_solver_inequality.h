#ifndef CVC5__THEORY__BV__BV_SOLVER_INEQUALITY_H
#define CVC5__THEORY__BV__BV_SOLVER_INEQUALITY_H
#include "theory/bv/bv_solver_inequality.h"
#include "theory/bv/bv_solver.h"
#include "theory/uf/equality_engine.h"
#include "theory/bv/bv_inequality_graph.h"

namespace cvc5::internal {

namespace theory {
namespace bv {

class BVSolverInEquality : public BVSolver
{
  InequalityGraph d_inequalityGraph;
  std::unordered_set<Node> d_ineqTerms;
  context::CDO<uint32_t> d_assertionIndex;
  context::CDQueue<TNode> d_assertionQueue;
 public:
  BVSolverInEquality(Env& env,
                   TheoryState& state,
                   TheoryInferenceManager& inferMgr):
    BVSolver(env, state, inferMgr),
    d_assertionIndex(env.getContext()),
    d_assertionQueue(env.getContext(), userContext()),
    d_inequalityGraph(env.getContext(), userContext()),
    d_ineqTerms()
    {}
        
  ~BVSolverInEquality() = default;


  void setEqualityEngine(eq::EqualityEngine* ee) override;

  bool preCheck(Theory::Effort level) override;

  TrustNode explain(TNode n) override;

  bool collectModelValues(TheoryModel* m,
                          const std::set<Node>& termSet) override;


  bool addInequality(TNode a, TNode b, bool strict, TNode fact);
  void preRegisterTerm(TNode node) override;
  bool done() { return d_assertionQueue.size() == d_assertionIndex; }
  TNode get() {
    Assert(!done());
    TNode res = d_assertionQueue[d_assertionIndex];
    d_assertionIndex = d_assertionIndex + 1;
    return res;
  }

 private:
  eq::EqualityEngine* d_ee;
};

}  // namespace bv
}  // namespace theory
}  // namespace cvc5::internal

#endif /* CVC5__THEORY__BV__BV_SOLVER_INEQUALITY_H */
