; COMMAND-LINE: --incremental
(set-logic ALL)
(set-option :incremental true)
(define-sort S16 () Float64)
(define-sort S37 () (Array S16 Int))
(declare-const c S37)
(assert (= 0 (select c (fp.min (fp (_ bv0 1) (_ bv0 11) (_ bv0 52)) (fp (_ bv1 1) (_ bv0 11) (_ bv0 52))))))
(set-info :status sat)
(check-sat)
(declare-const x Float64)
(declare-const __ (_ BitVec 1))
(assert (= (fp.to_real x) (fp.to_real (fp (_ bv0 1) (_ bv0 11) ((_ zero_extend 51) __)))))
(set-info :status sat)
(check-sat)