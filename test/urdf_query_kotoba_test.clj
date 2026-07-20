(ns urdf-query-kotoba-test
  (:require [clojure.java.io :as io]
            [clojure.java.shell :as shell]
            [clojure.string :as str]
            [clojure.test :refer [deftest is testing]]
            [kami_articulated :as articulated]
            [kotoba.compiler.core :as compiler]
            [kotoba.compiler.ir :as ir]))

(defn- js-array [values]
  (str "[" (str/join "," (map pr-str values)) "]"))

(defn- compiler-browser-host-url []
  (let [source (io/file (io/resource "kotoba/compiler/core.clj"))
        root (nth (iterate #(.getParentFile %) source) 4)
        host (io/file root "runtime/browser-host.mjs")]
    (when-not (.isFile host)
      (throw (ex-info "pinned compiler browser host is absent" {:path (.getPath host)})))
    (str (.toURI host))))

(defn- expected-query-values [system]
  {:robot-name (:name system)
   :link-names (mapv :name (:links system))
   :mass-values (->> (:links system)
                     (map #(get-in % [:inertia :mass]))
                     (remove zero?)
                     (mapv double))
   :joint-names (mapv :name (:joints system))
   :joint-types (mapv #(name (:kind %)) (:joints system))
   :joint-parents (mapv :parent (:joints system))
   :joint-children (mapv :child (:joints system))
   :limit-lower (mapv #(double (:lower %)) (:joints system))
   :limit-upper (mapv #(double (:upper %)) (:joints system))
   :limit-effort (mapv #(double (:effort %)) (:joints system))
   :limit-velocity (mapv #(double (:velocity %)) (:joints system))})

(defn- option-value [value]
  (when (true? (second value)) (nth value 2)))

(deftest numeric-query-fails-closed-as-typed-none
  (let [kir (:kir (compiler/compile-source (slurp "src/urdf_query.kotoba") :js-kotoba-v1))
        parse #(ir/execute kir 'mass-value [% 0])]
    (is (= [[:option :f64] false]
           (parse "<robot><link><inertial><mass/></inertial></link></robot>")))
    (is (= [[:option :f64] false]
           (parse "<robot><link><inertial><mass value=\"NaN\"/></inertial></link></robot>")))
    (is (= [[:option :f64] false]
           (parse "<robot><link><inertial><mass value=\"1e309\"/></inertial></link></robot>")))
    (is (= [[:option :f64] true -0.0]
           (parse "<robot><link><inertial><mass value=\"-0\"/></inertial></link></robot>")))))

(deftest real-urdf-fixtures-agree-across-cljc-reference-script-and-typed-wasm
  (let [source (slurp "src/urdf_query.kotoba")
        js-artifact (compiler/compile-source source :js-kotoba-v1)
        wasm-artifact (compiler/compile-source source :wasm32-browser-kotoba-v1)
        js64 (.encodeToString (java.util.Base64/getEncoder)
                              (.getBytes ^String (:source js-artifact) "UTF-8"))
        wasm64 (.encodeToString (java.util.Base64/getEncoder) (:bytes wasm-artifact))]
    (doseq [fixture ["cartpole.urdf" "hizukue.urdf"]]
      (testing fixture
        (let [xml (slurp (str "test/fixtures/" fixture))
              system (articulated/parse-urdf xml)
              expected (expected-query-values system)
              execute #(ir/execute (:kir js-artifact) %1 %2)
              reference
              {:robot-name (option-value (execute 'robot-name [xml]))
               :link-names (mapv #(option-value (execute 'link-name [xml %]))
                                 (range (count (:link-names expected))))
               :mass-values (mapv #(option-value (execute 'mass-value [xml %]))
                                  (range (count (:mass-values expected))))
               :joint-names (mapv #(option-value (execute 'joint-name [xml %]))
                                  (range (count (:joint-names expected))))
               :joint-types (mapv #(option-value (execute 'joint-type [xml %]))
                                  (range (count (:joint-types expected))))
               :joint-parents (mapv #(option-value (execute 'joint-parent [xml %]))
                                    (range (count (:joint-parents expected))))
               :joint-children (mapv #(option-value (execute 'joint-child [xml %]))
                                     (range (count (:joint-children expected))))
               :limit-lower (mapv #(option-value (execute 'limit-lower [xml %]))
                                  (range (count (:limit-lower expected))))
               :limit-upper (mapv #(option-value (execute 'limit-upper [xml %]))
                                  (range (count (:limit-upper expected))))
               :limit-effort (mapv #(option-value (execute 'limit-effort [xml %]))
                                   (range (count (:limit-effort expected))))
               :limit-velocity (mapv #(option-value (execute 'limit-velocity [xml %]))
                                     (range (count (:limit-velocity expected))))}
              xml64 (.encodeToString (java.util.Base64/getEncoder)
                                     (.getBytes ^String xml "UTF-8"))
              node-source
              (str "const xml=Buffer.from(process.argv[1],'base64').toString(),expected={"
                   "robot:" (pr-str (:robot-name expected)) ",links:" (js-array (:link-names expected))
                   ",masses:" (js-array (:mass-values expected)) ",joints:" (js-array (:joint-names expected))
                   ",types:" (js-array (:joint-types expected)) ",parents:" (js-array (:joint-parents expected))
                   ",children:" (js-array (:joint-children expected))
                   ",lower:" (js-array (:limit-lower expected)) ",upper:" (js-array (:limit-upper expected))
                   ",effort:" (js-array (:limit-effort expected)) ",velocity:" (js-array (:limit-velocity expected)) "};"
                   "const read=(x,n,count)=>Array.from({length:count},(_,i)=>{const v=x[n](xml,BigInt(i));if(!v[1])throw Error(n+' missing '+i);return v[2]});"
                   "const check=x=>{const robot=x['robot-name'](xml);if(!robot[1]||robot[2]!==expected.robot)throw Error('robot');"
                   "for(const [count,n,want] of [[x['link-count'](xml),'link-name',expected.links],[x['mass-count'](xml),'mass-value',expected.masses],[x['joint-count'](xml),'joint-name',expected.joints]])"
                   "if(count!==BigInt(want.length)||JSON.stringify(read(x,n,want.length))!==JSON.stringify(want))throw Error(n);"
                   "for(const [n,want] of [['joint-type',expected.types],['joint-parent',expected.parents],['joint-child',expected.children]])"
                   "if(JSON.stringify(read(x,n,want.length))!==JSON.stringify(want))throw Error(n);"
                   "if(x['limit-count'](xml)!==BigInt(expected.lower.length))throw Error('limit-count');"
                   "for(const [n,want] of [['limit-lower',expected.lower],['limit-upper',expected.upper],['limit-effort',expected.effort],['limit-velocity',expected.velocity]])"
                   "if(JSON.stringify(read(x,n,want.length))!==JSON.stringify(want))throw Error(n)};"
                   "Promise.all([import('data:text/javascript;base64," js64 "'),import(" (pr-str (compiler-browser-host-url)) ")]).then(async([j,h])=>{"
                   "check(j.instantiateKotoba({}));const w=await h.instantiateKotoba(Buffer.from('" wasm64 "','base64'));check(w.instance.exports)})"
                   ".catch(e=>{console.error(e);process.exit(70)})")
              node-result (shell/sh "node" "--input-type=module" "-e" node-source xml64)]
          (is (= expected reference))
          (is (zero? (:exit node-result)) (:err node-result)))))))
