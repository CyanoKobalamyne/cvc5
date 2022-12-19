#include "theory/bv/bv_solver_inequality.h"

#include <cstddef>
#include <unordered_map>
#include <vector>
#include "theory/theory.h"
#include "expr/type_node.h"
#include "theory/bv/theory_bv_utils.h"
#include "theory/theory_inference_manager.h"
#include "theory/uf/equality_engine_iterator.h"
#include "util/bitvector.h"

namespace cvc5::internal {
namespace theory {
namespace bv {


void BVSolverInEquality::setEqualityEngine(eq::EqualityEngine* ee) { d_ee = ee; }
bool BVSolverInEquality::done(){
  return d_facts.size()==d_factsHead;
 }
TNode BVSolverInEquality::get(){
    TNode res = d_facts[d_factsHead];
    d_factsHead = d_factsHead + 1;
    return res; }
bool BVSolverInEquality::preCheck(Theory::Effort level)
{
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
    // not ok
    std::vector<TNode> conflict;
    d_inequalityGraph.getConflict(conflict);
    Node confl = utils::flattenAnd(conflict);
    return true;
  }
  // skip check, return early
  return false;
 }

TrustNode BVSolverInEquality::explain(TNode n) { return TrustNode::null(); }

bool BVSolverInEquality::collectModelValues(TheoryModel* m,
                                          const std::set<Node>& termSet)
{
  return true;
}


bool BVSolverInEquality::addInequality(TNode a, TNode b, bool strict, TNode fact)
{
  bool ok = d_inequalityGraph.addInequality(a, b, strict, fact);
  if (!ok || !strict) return ok;
  return ok;
}

}  // namespace bv
}  // namespace theory
}  // namespace cvc5::internal
