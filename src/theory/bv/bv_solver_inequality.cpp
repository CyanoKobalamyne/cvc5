#include "theory/bv/bv_solver_inequality.h"
#include "options/bv_options.h"
#include "prop/sat_solver_factory.h"
#include "theory/bv/theory_bv.h"
#include "theory/bv/theory_bv_utils.h"
#include "theory/theory_model.h"

namespace cvc5::internal {
namespace theory {
namespace bv {

bool InequalitySolver::check(Theory::Effort e) {
  bool ok = true;
  while (!done() && ok) {
    TNode fact = get();
    if (fact.getKind() == kind::EQUAL) {
      TNode a = fact[0];
      if( a.getType().isBitVector() ){
        TNode b = fact[1];
        ok = addInequality(a, b, false, fact);
        if (ok)
          ok = addInequality(b, a, false, fact);
      }
    } else if (fact.getKind() == kind::NOT && fact[0].getKind() == kind::EQUAL) {
      TNode a = fact[0][0];
      if( a.getType().isBitVector() ){
        TNode b = fact[0][1];
        ok = d_inequalityGraph.addDisequality(a, b, fact);
      }
    }
    if (fact.getKind() == kind::NOT && fact[0].getKind() == kind::BITVECTOR_ULE) {
      TNode a = fact[0][1];
      TNode b = fact[0][0];
      ok = addInequality(a, b, true, fact);
    } else if (fact.getKind() == kind::NOT && fact[0].getKind() == kind::BITVECTOR_ULT) {
      TNode a = fact[0][1];
      TNode b = fact[0][0];
      ok = addInequality(a, b, false, fact);
    } else if (fact.getKind() == kind::BITVECTOR_ULT) {
      TNode a = fact[0];
      TNode b = fact[1];
      ok = addInequality(a, b, true, fact);
    } else if (fact.getKind() == kind::BITVECTOR_ULE) {
      TNode a = fact[0];
      TNode b = fact[1];
      ok = addInequality(a, b, false, fact);
    }
  }

  if (!ok) {
    std::vector<TNode> conflict;
    d_inequalityGraph.getConflict(conflict);
    Node confl = utils::flattenAnd(conflict);
    return false;
  }

  return true;
}

EqualityStatus InequalitySolver::getEqualityStatus(TNode a, TNode b)
{
  if (!isComplete()) return EQUALITY_UNKNOWN;

  NodeManager* nm = NodeManager::currentNM();
  Node a_lt_b = nm->mkNode(kind::BITVECTOR_ULT, a, b);
  Node b_lt_a = nm->mkNode(kind::BITVECTOR_ULT, b, a);

  if (d_assertionSet.contains(a_lt_b) || d_assertionSet.contains(b_lt_a))
  {
    return EQUALITY_FALSE;
  }

  if (!d_inequalityGraph.hasValueInModel(a)
      || !d_inequalityGraph.hasValueInModel(b))
  {
    return EQUALITY_UNKNOWN;
  }

  BitVector a_val = d_inequalityGraph.getValueInModel(a);
  BitVector b_val = d_inequalityGraph.getValueInModel(b);

  if (a_val == b_val)
  {
    return EQUALITY_TRUE_IN_MODEL;
  }
  else
  {
    return EQUALITY_FALSE_IN_MODEL;
  }
}

void InequalitySolver::assertFact(TNode fact) {
  d_assertionQueue.push_back(fact);
  d_assertionSet.insert(fact);
  if (!isInequalityOnly(fact)) {
    d_isComplete = false;
  }
}

bool InequalitySolver::isInequalityOnly(TNode node) {
  if (node.getKind() == kind::NOT) {
    node = node[0];
  }

  if (node.getAttribute(IneqOnlyComputedAttribute())) {
    return node.getAttribute(IneqOnlyAttribute());
  }

  if (node.getKind() != kind::EQUAL &&
      node.getKind() != kind::BITVECTOR_ULT &&
      node.getKind() != kind::BITVECTOR_ULE &&
      node.getKind() != kind::CONST_BITVECTOR &&
      node.getKind() != kind::SELECT &&
      node.getKind() != kind::STORE &&
      node.getMetaKind() != kind::metakind::VARIABLE) {
    return false;
  }
  bool res = true;
  for (unsigned i = 0; res && i < node.getNumChildren(); ++i) {
    res = res && isInequalityOnly(node[i]);
  }
  node.setAttribute(IneqOnlyComputedAttribute(), true);
  node.setAttribute(IneqOnlyAttribute(), res);
  return res;
}

void InequalitySolver::explain(TNode literal, std::vector<TNode>& assumptions) {
  Assert(d_explanations.find(literal) != d_explanations.end());
  TNode explanation = d_explanations[literal];
  assumptions.push_back(explanation);
}

void InequalitySolver::propagate(Theory::Effort e) { Assert(false); }
bool InequalitySolver::collectModelInfo(TheoryModel* m, bool fullModel)
{
  std::vector<Node> model;
  d_inequalityGraph.getAllValuesInModel(model);
  for (unsigned i = 0; i < model.size(); ++i) {
    Assert(model[i].getKind() == kind::EQUAL);
    if (!m->assertEquality(model[i][0], model[i][1], true))
    {
      return false;
    }
  }
  return true;
}

Node InequalitySolver::getModelValue(TNode var) {
  Assert(isInequalityOnly(var));
  Assert(isComplete());
  Node result = Node();
  if (!d_inequalityGraph.hasValueInModel(var)) {
  } else {
    BitVector val = d_inequalityGraph.getValueInModel(var);
    result = utils::mkConst(val);
  }
  return result;
}

void InequalitySolver::preRegister(TNode node) {
  Kind kind = node.getKind(); 
  if (kind == kind::EQUAL ||
      kind == kind::BITVECTOR_ULE ||
      kind == kind::BITVECTOR_ULT) {
    d_ineqTerms.insert(node[0]);
    d_ineqTerms.insert(node[1]);
  }
}

bool InequalitySolver::addInequality(TNode a, TNode b, bool strict, TNode fact)
{
  bool ok = d_inequalityGraph.addInequality(a, b, strict, fact);
  if (!ok || !strict) return ok;
  return ok;
}

}}}