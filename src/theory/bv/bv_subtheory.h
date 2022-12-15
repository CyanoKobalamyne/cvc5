/*********************                                                    
 ** Interface for bit-vectors sub-solvers.
 **/

#ifndef CVC5__THEORY__BV__BV_SUBTHEORY_H
#define CVC5__THEORY__BV__BV_SUBTHEORY_H

#include "cvc5_private.h"
#include "context/context.h"
#include "context/cdqueue.h"
#include "theory/uf/equality_engine.h"
#include "theory/theory.h"

namespace cvc5::internal {


namespace theory {

class TheoryModel;

namespace bv {
class TheoryBV;

using AssertionQueue = context::CDQueue<Node>;

/**
 * base class for bv sub-solvers
 *
 */
class SubtheorySolver {
 public:
  SubtheorySolver(context::Context* c, TheoryBV* bv)
      : d_context(c),
        d_bv(bv),
        d_assertionQueue(c),
        d_assertionIndex(c, 0) {}
  virtual ~SubtheorySolver() {}
  virtual bool check(Theory::Effort e) = 0;
  virtual void explain(TNode literal, std::vector<TNode>& assumptions) = 0;
  virtual void preRegister(TNode node) {}
  virtual void propagate(Theory::Effort e) {}
  virtual bool collectModelInfo(TheoryModel* m, bool fullModel) = 0;
  virtual Node getModelValue(TNode var) = 0;
  virtual bool isComplete() = 0;
  virtual EqualityStatus getEqualityStatus(TNode a, TNode b) = 0;
  virtual void addSharedTerm(TNode node) {}
  bool done() { return d_assertionQueue.size() == d_assertionIndex; }
  TNode get() {
    Assert(!done());
    TNode res = d_assertionQueue[d_assertionIndex];
    d_assertionIndex = d_assertionIndex + 1;
    return res;
  }
  virtual void assertFact(TNode fact) { d_assertionQueue.push_back(fact); }
  AssertionQueue::const_iterator assertionsBegin() {
    return d_assertionQueue.begin();
  }
  AssertionQueue::const_iterator assertionsEnd() {
    return d_assertionQueue.end();
  }

 protected:
  context::Context* d_context;

  TheoryBV* d_bv;
  AssertionQueue d_assertionQueue;
  context::CDO<uint32_t> d_assertionIndex;
}; 

}  // namespace bv
}  // namespace theory
}  // namespace CVC5

#endif /* CVC5__THEORY__BV__BV_SUBTHEORY_H */
