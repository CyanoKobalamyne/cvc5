/*
 ** Inequality subsolver
 **/

#include "cvc5_private.h"

#ifndef CVC5__THEORY__BV__BV_SUBTHEORY__INEQUALITY_H
#define CVC5__THEORY__BV__BV_SUBTHEORY__INEQUALITY_H

#include <unordered_set>

#include "context/cdhashset.h"
#include "expr/attribute.h"
#include "theory/bv/bv_inequality_graph.h"
#include "theory/bv/bv_subtheory.h"

namespace cvc5::internal {
namespace theory {
namespace bv {

struct IneqOnlyAttributeId {};
typedef expr::Attribute<IneqOnlyAttributeId, bool> IneqOnlyAttribute;

struct IneqOnlyComputedAttributeId {};
typedef expr::Attribute<IneqOnlyComputedAttributeId, bool> IneqOnlyComputedAttribute;

class InequalitySolver : public SubtheorySolver
{
  context::CDHashSet<Node > d_assertionSet;
  InequalityGraph d_inequalityGraph;
  context::CDHashMap<Node, TNode > d_explanations;
  context::CDO<bool> d_isComplete;
  typedef std::unordered_set<Node > NodeSet;
  NodeSet d_ineqTerms;
  bool isInequalityOnly(TNode node);
  bool addInequality(TNode a, TNode b, bool strict, TNode fact);
public:
  InequalitySolver(context::Context* c, context::Context* u, TheoryBV* bv)
    : SubtheorySolver(c, bv),
      d_assertionSet(c),
      d_inequalityGraph(c, u),
      d_explanations(c),
      d_isComplete(c, true),
      d_ineqTerms()
      {}
  

  
  bool check(Theory::Effort e) override;
  void propagate(Theory::Effort e) override;
  void explain(TNode literal, std::vector<TNode>& assumptions) override;
  bool isComplete() override { return d_isComplete; }
  bool collectModelInfo(TheoryModel* m, bool fullModel) override;
  Node getModelValue(TNode var) override;
  EqualityStatus getEqualityStatus(TNode a, TNode b) override;
  void assertFact(TNode fact) override;
  void preRegister(TNode node) override;


};
}
}}

#endif /* CVC5__THEORY__BV__BV_SUBTHEORY__INEQUALITY_H */
