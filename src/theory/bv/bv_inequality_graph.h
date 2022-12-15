/*********************                                                        */
/*inequality subsolver
 **/

#include "cvc5_private.h"

#ifndef CVC5__THEORY__BV__BV_INEQUALITY__GRAPH_H
#define CVC5__THEORY__BV__BV_INEQUALITY__GRAPH_H

#include <list>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "context/cdqueue.h"
#include "context/context.h"
#include "context/cdo.h"
#include "context/cdhashset.h"
#include "theory/theory.h"
#include "theory/uf/equality_engine.h"
#include "util/bitvector.h"


namespace cvc5::internal {
namespace theory {
namespace bv {

typedef unsigned TermId;
typedef unsigned ReasonId;
extern const TermId UndefinedTermId;
extern const ReasonId UndefinedReasonId;
extern const ReasonId AxiomReasonId;

class InequalityGraph : public context::ContextNotifyObj{

  struct InequalityEdge {
    TermId next;
    ReasonId reason;
    bool strict;
    InequalityEdge(TermId n, bool s, ReasonId r)
      : next(n),
        reason(r),
        strict(s)
    {}
    bool operator==(const InequalityEdge& other) const {
      return next == other.next && reason == other.reason && strict == other.strict; 
    }
  };

  class InequalityNode {
    TermId d_id;
    unsigned d_bitwidth;
    bool d_isConstant;
  public:
    InequalityNode(TermId id, unsigned bitwidth, bool isConst)
      : d_id(id),
        d_bitwidth(bitwidth),
        d_isConstant(isConst)
    {}
    TermId getId() const { return d_id; }
    unsigned getBitwidth() const { return d_bitwidth; }
    bool isConstant() const { return d_isConstant; }
  };

  struct ModelValue {
    TermId parent;
    ReasonId reason;
    BitVector value; 
    ModelValue()
      : parent(UndefinedTermId),
        reason(UndefinedReasonId),
        value(0, 0u)
    {}
    
    ModelValue(const BitVector& val, TermId p, ReasonId r)
      : parent(p),
        reason(r),
        value(val)
    {}
  };
  
  typedef context::CDHashMap<TermId, ModelValue> ModelValues;

  struct QueueComparator {
    const ModelValues* d_model;
    QueueComparator(const ModelValues* model)
      : d_model(model)
    {}
    bool operator() (TermId left, TermId right) const {
      Assert(d_model->find(left) != d_model->end()
             && d_model->find(right) != d_model->end());

      return (*(d_model->find(left))).second.value < (*(d_model->find(right))).second.value; 
    }
  }; 

  typedef std::unordered_map<TNode, ReasonId > ReasonToIdMap;
  typedef std::unordered_map<TNode, TermId > TermNodeToIdMap;

  typedef std::vector<InequalityEdge> Edges; 
  typedef std::unordered_set<TermId> TermIdSet;

  typedef std::priority_queue<TermId, std::vector<TermId>, QueueComparator> BFSQueue; 
  typedef std::unordered_set<TNode > TNodeSet;
  typedef std::unordered_set<Node > NodeSet;

  std::vector<InequalityNode> d_ineqNodes;
  std::vector< Edges > d_ineqEdges;

  NodeSet d_reasonSet; 
  std::vector<TNode> d_reasonNodes;
  ReasonToIdMap d_reasonToIdMap;
  
  std::vector<Node> d_termNodes;
  TermNodeToIdMap d_termNodeToIdMap;

  context::CDO<bool> d_inConflict;
  std::vector<TNode> d_conflict;

  ModelValues  d_modelValues;
  void initializeModelValue(TNode node); 
  void setModelValue(TermId term, const ModelValue& mv);
  ModelValue getModelValue(TermId term) const;
  bool hasModelValue(TermId id) const; 
  bool hasReason(TermId id) const; 
  
  TermId registerTerm(TNode term);
  TNode getTermNode(TermId id) const; 
  TermId getTermId(TNode node) const;
  bool isRegistered(TNode term) const; 
  
  ReasonId registerReason(TNode reason);
  TNode getReasonNode(ReasonId id) const;

  Edges& getEdges(TermId id)
  {
    Assert(id < d_ineqEdges.size());
    return d_ineqEdges[id];
  }
  InequalityNode& getInequalityNode(TermId id)
  {
    Assert(id < d_ineqNodes.size());
    return d_ineqNodes[id];
  }
  const InequalityNode& getInequalityNode(TermId id) const
  {
    Assert(id < d_ineqNodes.size());
    return d_ineqNodes[id];
  }
  unsigned getBitwidth(TermId id) const { return getInequalityNode(id).getBitwidth(); }
  bool isConst(TermId id) const { return getInequalityNode(id).isConstant(); }
  
  BitVector getValue(TermId id) const; 
    
  void addEdge(TermId a, TermId b, bool strict, TermId reason);
  
  void setConflict(const std::vector<ReasonId>& conflict);

  bool updateValue(TermId id, ModelValue new_mv, TermId start, bool& changed);
 
  bool processQueue(BFSQueue& queue, TermId start);

  void computeExplanation(TermId from, TermId to, std::vector<ReasonId>& explanation); 

  context::CDQueue<TNode> d_disequalities;
  typedef context::CDHashSet<Node > CDNodeSet;
  CDNodeSet d_disequalitiesAlreadySplit; 
  Node makeDiseqSplitLemma(TNode diseq); 
  /** Backtracking mechanisms **/
  std::vector<std::pair<TermId, InequalityEdge> > d_undoStack;
  context::CDO<unsigned> d_undoStackIndex;

  void contextNotifyPop() override { backtrack(); }

  void backtrack(); 

public:
  
  InequalityGraph(context::Context* c, context::Context* u, bool s = false)
    : ContextNotifyObj(c), 
      d_ineqNodes(),
      d_ineqEdges(),
      d_inConflict(c, false),
      d_conflict(),
      d_modelValues(c),
      d_disequalities(c),
      d_disequalitiesAlreadySplit(u),
      d_undoStack(),
      d_undoStackIndex(c)
  {}

  bool addInequality(TNode a, TNode b, bool strict, TNode reason);
  bool addDisequality(TNode a, TNode b, TNode reason); 
  void getConflict(std::vector<TNode>& conflict);
  virtual ~InequalityGraph() {}
  bool isLessThan(TNode a, TNode b);
  bool hasValueInModel(TNode a) const;
  BitVector getValueInModel(TNode a) const;
  void getAllValuesInModel(std::vector<Node>& assignments); 
}; 

}
}
}

#endif /* CVC5__THEORY__BV__BV_INEQUALITY__GRAPH_H */
