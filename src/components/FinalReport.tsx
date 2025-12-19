// src/components/FinalReport.tsx

import React, { useState, useEffect } from 'react';
import type { ProductType, WellConfig } from '../types';
import { RotateCcw, Printer, CheckCircle2, Save } from 'lucide-react';
// import logoImage from '../assets/logo.png'; // (참고) 로고 이미지가 있다면 경로 수정 필요

// ▼▼▼ [Logic] Firebase 라이브러리 추가 ▼▼▼
import { ref, update, get, set, child } from "firebase/database";
import { db } from "../../lib/firebase";

interface FinalReportProps {
  productType: ProductType;
  wellConfigs: WellConfig[];
  onRestart: () => void;
}

export function FinalReport({ productType, wellConfigs, onRestart }: FinalReportProps) {
  const [isSaved, setIsSaved] = useState(false);
  const [batchId] = useState(`LUMINA-${new Date().getFullYear()}${String(new Date().getMonth() + 1).padStart(2, '0')}${String(new Date().getDate()).padStart(2, '0')}-${String(Date.now()).slice(-6)}`);

  const currentDate = new Date().toLocaleDateString('en-US', { 
    year: 'numeric', month: 'long', day: 'numeric', hour: '2-digit', minute: '2-digit' 
  });

  // ▼▼▼ [Logic] 1. 자동 저장 로직 (History) ▼▼▼
  useEffect(() => {
    const saveHistory = async () => {
      try {
        const snapshot = await get(child(ref(db), 'current_order'));
        if (snapshot.exists()) {
          const orderData = snapshot.val();
          if (orderData.order_id) {
            const historyData = {
              ...orderData,
              completed_at: Date.now(),
              batch_id: batchId,
              final_result: "Success"
            };
            await set(ref(db, `history/${orderData.order_id}`), historyData);
            setIsSaved(true);
            console.log(`✅ History saved: ${orderData.order_id}`);
          }
        }
      } catch (error) {
        console.error("Failed to save history:", error);
      }
    };
    saveHistory();
  }, [batchId]);

  // ▼▼▼ [Logic] 2. 시스템 리셋 및 재시작 ▼▼▼
  const handleRestartSystem = () => {
    const updates: any = {};
    updates['current_order/status'] = 'waiting';
    updates['robot_state/current_action'] = 'Ready';
    updates['robot_state/progress_percent'] = 0;
    updates['robot_state/current_well'] = 0;

    update(ref(db), updates)
      .then(() => {
        console.log("🔄 System Reset Complete");
        onRestart();
      })
      .catch((err) => {
        console.error("Reset Failed", err);
        onRestart();
      });
  };

  const handlePrint = () => {
    window.print();
  };

  // Helper to simulate a color for display purposes based on mix
  const getSimulatedColor = (config: WellConfig) => {
     const total = config.powder1 + config.powder2 + config.liquid1 + config.liquid2;
     if (total === 0) return '#EAE8E4';
     
     // Mix logic simulation (simplified)
     const r = Math.min(255, 230 - (config.powder2 * 20) + (config.liquid2 * 10));
     const g = Math.min(255, 200 - (config.powder2 * 30) + (config.liquid2 * 5));
     const b = Math.min(255, 180 - (config.powder2 * 40));
     
     return `rgb(${r}, ${g}, ${b})`;
  };

  // Calculate an "Average" final color for the batch preview
  const averageColor = wellConfigs.reduce((acc, curr, _, arr) => {
      const col = getSimulatedColor(curr);
      return col !== '#EAE8E4' ? col : acc;
  }, '#E6DCCF');

  return (
    <div className="min-h-screen bg-[#D8C3BE] p-4 md:p-8 font-sans flex items-center justify-center">
      <div className="max-w-5xl w-full flex flex-col gap-8">
        
        {/* Paper Document Container */}
        <div className="bg-[#FAF9F6] rounded-[1rem] shadow-xl overflow-hidden relative min-h-[800px] flex flex-col border border-white/40">
           {/* Top Color Bar */}
           <div className="h-3 bg-[#1C1917] w-full" />
           
           {/* Report Header */}
           <div className="p-12 pb-8 border-b border-[#1C1917]/10 flex justify-between items-start">
              <div>
                 {/* 로고 이미지가 없다면 텍스트로 대체하거나 주석 처리 */}
                 {/* <div className="mb-6 w-48 opacity-90">
                    <img src={logoImage} alt="BEAURO" className="w-full grayscale contrast-125" />
                 </div> */}
                 <h1 className="text-3xl font-bold text-[#1C1917] tracking-tight mb-2 font-serif uppercase">
                    Formulation Certificate
                 </h1>
                 <p className="text-[#1C1917]/40 text-sm tracking-wide uppercase font-bold">Official Laboratory Report</p>
              </div>
              <div className="text-right hidden sm:block">
                 <div className="inline-block px-4 py-2 bg-[#1C1917] text-[#FAF9F6] rounded text-xs font-bold tracking-widest uppercase mb-2">
                    {isSaved ? "Saved to DB" : "Processing..."}
                 </div>
                 <p className="text-[#1C1917]/40 text-xs font-mono">ID: {batchId}</p>
                 <p className="text-[#1C1917]/40 text-xs font-mono">{currentDate}</p>
              </div>
           </div>

           {/* Report Body */}
           <div className="flex-1 p-12 grid grid-cols-12 gap-12">
              
              {/* Left Column: Specs */}
              <div className="col-span-12 md:col-span-4 space-y-10 border-r border-[#1C1917]/10 pr-12">
                 <div>
                    <h3 className="text-xs font-bold text-[#1C1917]/40 tracking-[0.2em] uppercase mb-4">Product Type</h3>
                    <p className="text-2xl font-serif text-[#1C1917] italic">
                       {productType === 'foundation' ? 'Liquid Foundation' : 'Lip Tint Serum'}
                    </p>
                 </div>

                 <div>
                    <h3 className="text-xs font-bold text-[#1C1917]/40 tracking-[0.2em] uppercase mb-4">Batch Parameters</h3>
                    <ul className="space-y-4 text-sm text-[#1C1917]/70 font-medium">
                       <li className="flex justify-between border-b border-[#1C1917]/5 pb-2">
                          <span>Viscosity Target</span>
                          <span className="font-bold text-[#1C1917]">2400 cP</span>
                       </li>
                       <li className="flex justify-between border-b border-[#1C1917]/5 pb-2">
                          <span>Homogenization</span>
                          <span className="font-bold text-[#1C1917]">3000 RPM</span>
                       </li>
                       <li className="flex justify-between border-b border-[#1C1917]/5 pb-2">
                          <span>Temperature</span>
                          <span className="font-bold text-[#1C1917]">22.5°C</span>
                       </li>
                       <li className="flex justify-between border-b border-[#1C1917]/5 pb-2">
                          <span>Humidity</span>
                          <span className="font-bold text-[#1C1917]">45%</span>
                       </li>
                    </ul>
                 </div>
                 
                 <div className="pt-8">
                    <h3 className="text-xs font-bold text-[#1C1917]/40 tracking-[0.2em] uppercase mb-4">Final Shade</h3>
                    <div className="relative inline-block">
                        <div 
                            className="w-24 h-24 rounded-full shadow-lg ring-4 ring-white"
                            style={{ backgroundColor: averageColor }}
                        />
                        <div className="absolute -bottom-2 -right-2 bg-[#1C1917] text-white text-[10px] font-bold px-2 py-1 rounded-full uppercase tracking-widest">
                            Result
                        </div>
                    </div>
                 </div>
              </div>

              {/* Right Column: Matrix Results */}
              <div className="col-span-12 md:col-span-8">
                 <h3 className="text-xs font-bold text-[#1C1917]/40 tracking-[0.2em] uppercase mb-8">Composition Analysis</h3>
                 
                 <div className="grid grid-cols-1 sm:grid-cols-2 gap-x-8 gap-y-6">
                    {wellConfigs.map((config, idx) => {
                       const total = config.powder1 + config.powder2 + config.liquid1 + config.liquid2;
                       const wellColor = getSimulatedColor(config);

                       return (
                          <div key={idx} className="flex items-center gap-4 p-4 bg-white rounded-lg border border-transparent hover:border-[#1C1917]/10 transition-colors shadow-sm">
                             <div 
                                className="w-10 h-10 rounded-full border border-stone-200 flex items-center justify-center font-bold font-serif shadow-sm text-[#1C1917]/60"
                                style={{ backgroundColor: wellColor }}
                             >
                                <span className="bg-white/90 px-1 rounded text-[10px]">0{idx + 1}</span>
                             </div>
                             <div className="flex-1">
                                <div className="h-2 w-full bg-[#EAE8E4] rounded-full overflow-hidden flex border border-[#1C1917]/5">
                                   <div style={{ flex: config.powder1, background: '#BFA082' }} />
                                   <div style={{ flex: config.powder2, background: '#8C705F' }} />
                                   <div style={{ flex: config.liquid1, background: '#E6DCCF' }} />
                                   <div style={{ flex: config.liquid2, background: '#D4AF37' }} />
                                   {total < 5 && <div style={{ flex: 5 - total, background: 'transparent' }} />}
                                </div>
                                <div className="mt-2 flex justify-between text-[10px] text-[#1C1917]/40 uppercase tracking-wider font-bold">
                                   <span>Composition</span>
                                   <span className="text-[#1C1917]">{total} Units</span>
                                </div>
                             </div>
                             <CheckCircle2 className="w-5 h-5 text-[#1C1917]" />
                          </div>
                       );
                    })}
                 </div>

                 <div className="mt-12 pt-8 border-t border-[#1C1917]/10 flex items-end justify-between">
                    <div>
                       <p className="font-serif text-lg italic text-[#1C1917]/80">"Quality is never an accident; it is always the result of high intention."</p>
                    </div>
                    <div className="text-right">
                       <div className="h-12 w-32 border-b border-[#1C1917]/20 mb-2 font-handwriting text-2xl text-[#1C1917] opacity-80" /> 
                       <p className="text-[10px] text-[#1C1917]/40 uppercase tracking-widest">Authorized Signature</p>
                    </div>
                 </div>
              </div>

           </div>
           
           {/* Decorative Bottom */}
           <div className="h-2 bg-[#EAE8E4] w-full mt-auto" />
        </div>

        {/* Action Buttons */}
        <div className="flex justify-center gap-4 animate-in fade-in slide-in-from-bottom-4 duration-700 delay-300 pb-10">
           <button 
              onClick={handlePrint}
              className="px-8 py-4 bg-[#FAF9F6] text-[#1C1917] rounded-xl font-bold tracking-widest uppercase hover:bg-white transition-colors shadow-lg hover:shadow-xl flex items-center gap-3"
           >
              <Printer className="w-4 h-4" />
              <span>Print Report</span>
           </button>
           <button 
              onClick={handleRestartSystem}
              className="px-8 py-4 bg-[#1C1917] text-white rounded-xl font-bold tracking-widest uppercase hover:bg-black transition-colors shadow-lg hover:shadow-xl flex items-center gap-3"
           >
              <RotateCcw className="w-4 h-4" />
              <span>New Batch</span>
           </button>
        </div>

      </div>
    </div>
  );
}