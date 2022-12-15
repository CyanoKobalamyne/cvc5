/*
 ** Inequality graph
 **/
#include "theory/bv/bv_inequality_graph.h"
#include "theory/bv/theory_bv_utils.h"
#include "context/cdqueue.h"

namespace cvc5::internal {
namespace theory {
namespace bv {
const TermId UndefinedTermId = -1; 
const ReasonId UndefinedReasonId = -1;
const ReasonId AxiomReasonId = -2;


bool InequalityGraph::addInequality(TNode a, TNode b, bool strict, TNode reason) {

  TermId id_a = registerTerm(a);
  TermId id_b = registerTerm(b);
  ReasonId id_reason = registerReason(reason);

  Assert(!(isConst(id_a) && isConst(id_b)));
  BitVector a_val = getValue(id_a);
  BitVector b_val = getValue(id_b);
    
  unsigned bitwidth = utils::getSize(a); 
  BitVector diff = strict ? BitVector(bitwidth, 1u) : BitVector(bitwidth, 0u);

  if (a_val + diff < a_val) {
    std::vector<ReasonId> conflict;
    conflict.push_back(id_reason);
    computeExplanation(UndefinedTermId, id_a, conflict);
    setConflict(conflict);
    return false; 
  }
  
  if (a_val + diff <= b_val) {
    addEdge(id_a, id_b, strict, id_reason);
    return true;
  }

//  in conflict
  if (isConst(id_b) && a_val + diff > b_val) {
    std::vector<ReasonId> conflict;
    conflict.push_back(id_reason);
    computeExplanation(UndefinedTermId, id_a, conflict);
    setConflict(conflict);
    return false; 
  }
  
  // add the inequality edge
  addEdge(id_a, id_b, strict, id_reason);
  BFSQueue queue(&d_modelValues);
  Assert(hasModelValue(id_a));
  queue.push(id_a);
  return processQueue(queue, id_a); 
}

bool InequalityGraph::updateValue(TermId id, ModelValue new_mv, TermId start, bool& changed) {
  BitVector lower_bound = new_mv.value;
  
  if (isConst(id)) {
    if (getValue(id) < lower_bound) {
      std::vector<ReasonId> conflict;
      TermId parent = new_mv.parent; 
      ReasonId reason = new_mv.reason; 
      conflict.push_back(reason); 
      computeExplanation(UndefinedTermId, parent, conflict);
      setConflict(conflict); 
      return false; 
    }
  } else {
    if (getValue(id) < lower_bound) {
      // cycle
      if (id == start) {
        TermId parent = new_mv.parent;
        ReasonId reason = new_mv.reason;
        std::vector<TermId> conflict;
        conflict.push_back(reason);
        computeExplanation(id, parent, conflict);
        setConflict(conflict); 
        return false; 
      }
      changed = true;
      setModelValue(id, new_mv); 
    }
  }
  return true; 
}

bool InequalityGraph::processQueue(BFSQueue& queue, TermId start) {
  while (!queue.empty()) {
    TermId current = queue.top();
    queue.pop();  
    BitVector current_value = getValue(current);
  
    unsigned size = getBitwidth(current);
    const BitVector zero(size, 0u); 
    const BitVector one(size, 1u); 
  
    const Edges& edges = getEdges(current);
    for (Edges::const_iterator it = edges.begin(); it!= edges.end(); ++it) {
      TermId next = it->next;
      ReasonId reason = it->reason;

      const BitVector increment = it->strict ? one : zero; 
      const BitVector next_lower_bound = current_value + increment;

      if (next_lower_bound < current_value) {
        std::vector<TermId> conflict;
        conflict.push_back(it->reason);
        Assert(hasModelValue(start));
        ReasonId start_reason = getModelValue(start).reason;
        if (start_reason != UndefinedReasonId) {
          conflict.push_back(start_reason);
        }
        computeExplanation(UndefinedTermId, current, conflict);
        setConflict(conflict); 
        return false; 
      }
      
      ModelValue new_mv(next_lower_bound, current, reason);       
      bool updated = false; 
      if (!updateValue(next, new_mv, start, updated)) {
        return false; 
      }
      
      if (next == start) {
        continue; 
      }
      
      if (!updated) {
        continue; 
      }

      queue.push(next);
    }
  }
  return true; 
}

void InequalityGraph::computeExplanation(TermId from, TermId to, std::vector<ReasonId>& explanation) {

  TermIdSet seen;

  while(hasReason(to) && from != to && !seen.count(to)) {
    seen.insert(to); 
    const ModelValue& exp = getModelValue(to);
    Assert(exp.reason != UndefinedReasonId);
    explanation.push_back(exp.reason);
    Assert(exp.parent != UndefinedTermId);
    to = exp.parent; 
  }
}

void InequalityGraph::addEdge(TermId a, TermId b, bool strict, TermId reason) {
  Edges& edges = getEdges(a);
  InequalityEdge new_edge(b, strict, reason); 
  edges.push_back(new_edge);
  d_undoStack.push_back(std::make_pair(a, new_edge));
  d_undoStackIndex = d_undoStackIndex + 1; 
}

void InequalityGraph::initializeModelValue(TNode node) {
  TermId id = getTermId(node);
  Assert(!hasModelValue(id));
  bool isConst = node.getKind() == kind::CONST_BITVECTOR;
  unsigned size = utils::getSize(node); 
  BitVector value = isConst? node.getConst<BitVector>() : BitVector(size, 0u); 
  setModelValue(id, ModelValue(value, UndefinedTermId, UndefinedReasonId));
}

bool InequalityGraph::isRegistered(TNode term) const {
  return d_termNodeToIdMap.find(term) != d_termNodeToIdMap.end(); 
}

TermId InequalityGraph::registerTerm(TNode term) {
  if (d_termNodeToIdMap.find(term) != d_termNodeToIdMap.end()) {
    TermId id = d_termNodeToIdMap[term];
    if (!hasModelValue(id)) {
      initializeModelValue(term); 
    }
    return id; 
  }

  TermId id = d_termNodes.size();
  
  d_termNodes.push_back(term);
  d_termNodeToIdMap[term] = id;
  
  unsigned size = utils::getSize(term);

  bool isConst = term.getKind() == kind::CONST_BITVECTOR;
  InequalityNode ineq = InequalityNode(id, size, isConst);

  Assert(d_ineqNodes.size() == id);
  d_ineqNodes.push_back(ineq);

  Assert(d_ineqEdges.size() == id);
  d_ineqEdges.push_back(Edges());

  initializeModelValue(term); 
  
  return id; 
}

ReasonId InequalityGraph::registerReason(TNode reason) {
  if (d_reasonToIdMap.find(reason) != d_reasonToIdMap.end()) {
    return d_reasonToIdMap[reason]; 
  }
  d_reasonSet.insert(reason);
  ReasonId id = d_reasonNodes.size();
  d_reasonNodes.push_back(reason);
  d_reasonToIdMap[reason] = id;
  return id; 
}

TNode InequalityGraph::getReasonNode(ReasonId id) const {
  Assert(d_reasonNodes.size() > id);
  return d_reasonNodes[id]; 
}

TNode InequalityGraph::getTermNode(TermId id) const {
  Assert(d_termNodes.size() > id);
  return d_termNodes[id]; 
}

TermId InequalityGraph::getTermId(TNode node) const {
  Assert(d_termNodeToIdMap.find(node) != d_termNodeToIdMap.end());
  return d_termNodeToIdMap.find(node)->second; 
}

void InequalityGraph::setConflict(const std::vector<ReasonId>& conflict) {
  Assert(!d_inConflict);
  d_inConflict = true;
  d_conflict.clear(); 
  for (unsigned i = 0; i < conflict.size(); ++i) {
    if (conflict[i] != AxiomReasonId) {
      d_conflict.push_back(getReasonNode(conflict[i]));
    }
  }
}

void InequalityGraph::getConflict(std::vector<TNode>& conflict) {
  for (unsigned i = 0; i < d_conflict.size(); ++i) {
    conflict.push_back(d_conflict[i]); 
  }
}

void InequalityGraph::setModelValue(TermId term, const ModelValue& mv) {
  d_modelValues[term] = mv; 
}

InequalityGraph::ModelValue InequalityGraph::getModelValue(TermId term) const {
  Assert(d_modelValues.find(term) != d_modelValues.end());
  return (*(d_modelValues.find(term))).second; 
}

bool InequalityGraph::hasModelValue(TermId id) const {
  return d_modelValues.find(id) != d_modelValues.end(); 
}

BitVector InequalityGraph::getValue(TermId id) const {
  Assert(hasModelValue(id));
  return (*(d_modelValues.find(id))).second.value;
}

bool InequalityGraph::hasReason(TermId id) const {
  const ModelValue& mv = getModelValue(id);
  return mv.reason != UndefinedReasonId; 
}

bool InequalityGraph::addDisequality(TNode a, TNode b, TNode reason) {
  d_disequalities.push_back(reason);

  if (!isRegistered(a) || !isRegistered(b)) {
    return true; 
  }
  TermId id_a = getTermId(a);
  TermId id_b = getTermId(b);
  if (!hasModelValue(id_a)) {
    initializeModelValue(a); 
  }
  if (!hasModelValue(id_b)) {
    initializeModelValue(b); 
  }
  const BitVector val_a = getValue(id_a);
  const BitVector val_b = getValue(id_b);
  if (val_a == val_b) {
    if (a.getKind() == kind::CONST_BITVECTOR) {
      std::vector<ReasonId> explanation_ids; 
      computeExplanation(UndefinedTermId, id_b, explanation_ids); 
      std::vector<TNode> explanation_nodes;
      explanation_nodes.push_back(reason);
      for (unsigned i = 0; i < explanation_ids.size(); ++i) {
        explanation_nodes.push_back(getReasonNode(explanation_ids[i])); 
      }
      Node explanation = utils::mkAnd(explanation_nodes);
      d_reasonSet.insert(explanation); 
      return addInequality(a, b, true, explanation);
    }
    if (b.getKind() == kind::CONST_BITVECTOR) {
      std::vector<ReasonId> explanation_ids; 
      computeExplanation(UndefinedTermId, id_a, explanation_ids); 
      std::vector<TNode> explanation_nodes;
      explanation_nodes.push_back(reason);
      for (unsigned i = 0; i < explanation_ids.size(); ++i) {
        explanation_nodes.push_back(getReasonNode(explanation_ids[i])); 
      }
      Node explanation = utils::mkAnd(explanation_nodes);
      d_reasonSet.insert(explanation); 
      return addInequality(b, a, true, explanation);
    }
  } 
  return true; 
}

void InequalityGraph::backtrack() {
  int size = d_undoStack.size(); 
  for (int i = size - 1; i >= (int)d_undoStackIndex.get(); --i) {
    Assert(!d_undoStack.empty());
    TermId id = d_undoStack.back().first; 
    d_undoStack.pop_back();
    
    Edges& edges = getEdges(id);

    edges.pop_back(); 
  }
}

Node InequalityGraph::makeDiseqSplitLemma(TNode diseq)
{
  Assert(diseq.getKind() == kind::NOT && diseq[0].getKind() == kind::EQUAL);
  NodeManager* nm = NodeManager::currentNM();
  TNode a = diseq[0][0];
  TNode b = diseq[0][1];
  Node a_lt_b = nm->mkNode(kind::BITVECTOR_ULT, a, b);
  Node b_lt_a = nm->mkNode(kind::BITVECTOR_ULT, b, a);
  Node eq = diseq[0];
  Node lemma = nm->mkNode(kind::OR, a_lt_b, b_lt_a, eq);
  return lemma;
}


bool InequalityGraph::isLessThan(TNode a, TNode b) {
  Assert(isRegistered(a) && isRegistered(b));
  Unimplemented(); 
}

bool InequalityGraph::hasValueInModel(TNode node) const {
  if (isRegistered(node)) {
    TermId id = getTermId(node);
    return hasModelValue(id); 
  }
  return false; 
}

BitVector InequalityGraph::getValueInModel(TNode node) const {
  TermId id = getTermId(node);
  Assert(hasModelValue(id));
  return getValue(id); 
}

void InequalityGraph::getAllValuesInModel(std::vector<Node>& assignments)
{
  NodeManager* nm = NodeManager::currentNM();
  for (ModelValues::const_iterator it = d_modelValues.begin();
       it != d_modelValues.end();
       ++it)
  {
    TermId id = (*it).first;
    BitVector value = (*it).second.value;
    TNode var = getTermNode(id);
    Node constant = utils::mkConst(value);
    Node assignment = nm->mkNode(kind::EQUAL, var, constant);
    assignments.push_back(assignment);
  }
}
}}}