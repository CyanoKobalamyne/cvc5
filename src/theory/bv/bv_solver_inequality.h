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
  context::CDO<uint32_t> d_factsHead;
  context::CDQueue<TNode> d_facts;
  InequalityGraph d_inequalityGraph;
  std::unordered_set<Node> d_ineqTerms;
 public:
  BVSolverInEquality(Env& env,
                   TheoryState& state,
                   TheoryInferenceManager& inferMgr):
    BVSolver(env, state, inferMgr),
    d_factsHead(env.getContext()),
    d_facts(env.getContext(), userContext()),
    d_inequalityGraph(env.getContext(), userContext()),
    d_ineqTerms()
    {}
        
  ~BVSolverInEquality() = default;

  std::string identify() const override { return "BVSolverInEquality"; };

  void setEqualityEngine(eq::EqualityEngine* ee) override;

  bool preCheck(Theory::Effort level) override;

  TrustNode explain(TNode n) override;

  bool collectModelValues(TheoryModel* m,
                          const std::set<Node>& termSet) override;


  bool addInequality(TNode a, TNode b, bool strict, TNode fact);
  void preRegisterTerm(TNode node) override;

  TNode get()  {
    TNode res = d_facts[d_factsHead];
    d_factsHead = d_factsHead + 1;
    return res;
  }

 private:
  eq::EqualityEngine* d_ee;
};

}  // namespace bv
}  // namespace theory
}  // namespace cvc5::internal

#endif /* CVC5__THEORY__BV__BV_SOLVER_INEQUALITY_H */
