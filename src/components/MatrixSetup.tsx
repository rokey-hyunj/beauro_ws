// src/components/MatrixSetup.tsx

import React, { useState } from 'react';
import { ChevronLeft, AlertCircle, Plus, Minus } from 'lucide-react';
import type { ProductType, WellConfig } from '../types';
import { RobotMonitor } from './RobotMonitor';
import { MonitorButton } from './MonitorButton';

// ▼▼▼ [Logic] Firebase 라이브러리 복구 ▼▼▼
import { ref, set } from "firebase/database";
import { db } from "../../lib/firebase";

interface MatrixSetupProps {
  productType: ProductType;
  initialConfigs: WellConfig[];
  onComplete: (configs: WellConfig[]) => void;
  onBack: () => void;
}

export function MatrixSetup({ productType, initialConfigs, onComplete, onBack }: MatrixSetupProps) {
  const [configs, setConfigs] = useState<WellConfig[]>(initialConfigs);
  const [selectedWell, setSelectedWell] = useState(0);
  const [showRobotMonitor, setShowRobotMonitor] = useState(false);

  const materials = [
    { key: 'powder1' as const, name: 'Powder A', color: '#BFA082' },
    { key: 'powder2' as const, name: 'Powder B', color: '#8C705F' },
    { key: 'liquid1' as const, name: 'Liquid A', color: '#E6DCCF' },
    { key: 'liquid2' as const, name: 'Liquid B', color: '#D4AF37' }
  ];

  const getCurrentTotal = (wellIndex: number) => {
    const config = configs[wellIndex];
    return config.powder1 + config.powder2 + config.liquid1 + config.liquid2;
  };

  const canIncrement = (wellIndex: number, material: keyof WellConfig) => {
    const config = configs[wellIndex];
    return config[material] < 3 && getCurrentTotal(wellIndex) < 5;
  };

  const handleIncrement = (wellIndex: number, material: keyof WellConfig) => {
    if (!canIncrement(wellIndex, material)) return;
    
    const newConfigs = [...configs];
    newConfigs[wellIndex] = {
      ...newConfigs[wellIndex],
      [material]: newConfigs[wellIndex][material] + 1
    };
    setConfigs(newConfigs);
  };

  const handleDecrement = (wellIndex: number, material: keyof WellConfig) => {
    const newConfigs = [...configs];
    if (newConfigs[wellIndex][material] > 0) {
      newConfigs[wellIndex] = {
        ...newConfigs[wellIndex],
        [material]: newConfigs[wellIndex][material] - 1
      };
      setConfigs(newConfigs);
    }
  };

  const loadPreset = (preset: 'gradient' | 'contrast' | 'balanced') => {
    let newConfigs: WellConfig[] = [];
    
    if (preset === 'gradient') {
      newConfigs = [
        { powder1: 3, powder2: 0, liquid1: 2, liquid2: 0 },
        { powder1: 2, powder2: 1, liquid1: 1, liquid2: 1 },
        { powder1: 2, powder2: 1, liquid1: 2, liquid2: 0 },
        { powder1: 1, powder2: 2, liquid1: 1, liquid2: 1 },
        { powder1: 1, powder2: 2, liquid1: 0, liquid2: 2 },
        { powder1: 0, powder2: 3, liquid1: 0, liquid2: 2 }
      ];
    } else if (preset === 'contrast') {
      newConfigs = [
        { powder1: 3, powder2: 0, liquid1: 2, liquid2: 0 },
        { powder1: 0, powder2: 3, liquid1: 0, liquid2: 2 },
        { powder1: 3, powder2: 0, liquid1: 2, liquid2: 0 },
        { powder1: 0, powder2: 3, liquid1: 0, liquid2: 2 },
        { powder1: 2, powder2: 1, liquid1: 1, liquid2: 1 },
        { powder1: 1, powder2: 2, liquid1: 1, liquid2: 1 }
      ];
    } else {
      newConfigs = Array(6).fill(null).map(() => ({
        powder1: 1,
        powder2: 1,
        liquid1: 1,
        liquid2: 1
      }));
    }
    
    setConfigs(newConfigs);
  };

  // ▼▼▼ [Logic] Firebase 전송 및 데이터 계산 함수 (이식 완료) ▼▼▼
  const UNIT_POWDER_MG = 20; 
  const UNIT_LIQUID_UL = 50; 

  const handleStartProduction = () => {
    // 1. 상세 데이터 계산
    const detailedMatrix = configs.map((config, index) => ({
      well_id: index + 1,
      
      counts: {
        p1: config.powder1,
        p2: config.powder2,
        l1: config.liquid1,
        l2: config.liquid2
      },

      amounts: {
        p1_mg: config.powder1 * UNIT_POWDER_MG,
        p2_mg: config.powder2 * UNIT_POWDER_MG,
        l1_ul: config.liquid1 * UNIT_LIQUID_UL,
        l2_ul: config.liquid2 * UNIT_LIQUID_UL
      },
      
      total_mass: (config.powder1 + config.powder2) * UNIT_POWDER_MG + 
                  (config.liquid1 + config.liquid2) * UNIT_LIQUID_UL
    }));

    // 2. 실험 ID 생성
    const dateStr = new Date().toISOString().slice(0, 10).replace(/-/g, "");
    const randomSuffix = Math.floor(Math.random() * 1000).toString().padStart(3, "0");
    const experimentID = `EXP_${dateStr}_${randomSuffix}`;

    const orderData = {
      order_id: experimentID,
      status: 'start',
      selected_type: productType,
      timestamp: Date.now(),
      doe_matrix: detailedMatrix
    };

    // 3. Firebase 전송
    set(ref(db, 'current_order'), orderData)
      .then(() => {
        alert(`🚀 실험 [${experimentID}] 시작!`);
        onComplete(configs); // 다음 화면으로 이동
      })
      .catch((error) => {
        console.error("Firebase Error:", error);
        alert("통신 오류가 발생했습니다.");
      });
  };
  // ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

  const isValid = configs.every(config => getCurrentTotal(configs.indexOf(config)) > 0);

  return (
    <div className="min-h-screen bg-[#D8C3BE] p-6 md:p-12 font-sans flex flex-col">
      <div className="max-w-[1400px] mx-auto w-full flex-1 flex flex-col">
        {/* Header */}
        <div className="flex items-start justify-between mb-10">
          <div className="flex items-center gap-6">
            <button
              onClick={onBack}
              className="p-3 bg-[#FAF9F6] rounded-full hover:shadow-lg transition-all text-[#1C1917]"
            >
              <ChevronLeft className="w-5 h-5" />
            </button>
            <div>
              <h1 className="text-3xl font-bold text-[#1C1917] tracking-tight mb-1 uppercase">
                Design Matrix
              </h1>
              <p className="text-[#1C1917]/50 font-serif italic">
                {productType === 'foundation' ? 'Foundation' : 'Lip Tint'} Formulation Setup
              </p>
            </div>
          </div>
          
          <div className="flex items-center gap-4">
            <div className="flex gap-2">
               {['gradient', 'contrast', 'balanced'].map(preset => (
                 <button
                   key={preset}
                   onClick={() => loadPreset(preset as any)}
                   className="px-4 py-2 rounded-full border border-[#1C1917]/20 text-[#1C1917]/60 
                            hover:border-[#1C1917] hover:text-[#1C1917] hover:bg-[#FAF9F6] 
                            transition-all text-xs font-bold tracking-widest uppercase"
                 >
                   {preset}
                 </button>
               ))}
            </div>
            
            <MonitorButton onClick={() => setShowRobotMonitor(true)} />
          </div>
        </div>

        <div className="flex-1 grid grid-cols-12 gap-8">
          
          {/* LEFT: TRAY VISUALIZATION */}
          <div className="col-span-12 lg:col-span-7 bg-[#FAF9F6] rounded-[2rem] p-10 shadow-sm relative overflow-hidden">
             <div className="absolute top-0 right-0 p-8 opacity-[0.03] pointer-events-none">
                <h1 className="text-[10rem] leading-none font-bold text-[#1C1917]">06</h1>
             </div>

             <div className="h-full flex flex-col">
                <div className="flex justify-between items-end mb-8 border-b border-[#1C1917]/10 pb-4">
                  <h3 className="text-xs font-bold text-[#1C1917]/40 tracking-[0.2em] uppercase">Tray Configuration</h3>
                  <span className="text-[#1C1917] font-mono text-xs">DoE Matrix v2.0</span>
                </div>

                <div className="flex-1 grid grid-cols-3 gap-6">
                  {configs.map((config, index) => {
                    const total = getCurrentTotal(index);
                    const isSelected = selectedWell === index;
                    
                    return (
                      <button
                        key={index}
                        onClick={() => setSelectedWell(index)}
                        className={`relative rounded-2xl transition-all duration-300 group
                                  flex flex-col overflow-hidden
                                  ${isSelected 
                                    ? 'ring-4 ring-[#1C1917] shadow-xl scale-[1.02] z-10' 
                                    : 'hover:shadow-lg hover:-translate-y-1 bg-white'}`}
                      >
                        {/* Material Stack Visualization */}
                        <div className="flex-1 w-full bg-[#EAE8E4] relative">
                          <div className="absolute inset-0 flex flex-col-reverse">
                             {[
                               { val: config.liquid2, col: materials[3].color },
                               { val: config.liquid1, col: materials[2].color },
                               { val: config.powder2, col: materials[1].color },
                               { val: config.powder1, col: materials[0].color }
                             ].map((item, idx) => (
                               item.val > 0 && (
                                 <div 
                                   key={idx}
                                   style={{ 
                                     flex: item.val,
                                     backgroundColor: item.col 
                                   }} 
                                   className="w-full transition-all duration-500 border-t border-white/20 first:border-0"
                                 />
                               )
                             ))}
                             {total === 0 && <div className="flex-1 bg-transparent" />}
                             {total < 5 && <div style={{ flex: 5 - total }} className="bg-transparent" />}
                          </div>

                          <div className="absolute top-4 left-4 z-10">
                            <span className={`text-3xl font-bold font-serif ${total > 2 ? 'text-white drop-shadow-md' : 'text-[#1C1917]/30'}`}>
                              0{index + 1}
                            </span>
                          </div>
                        </div>

                        {/* Info Footer */}
                        <div className={`p-4 flex justify-between items-center transition-colors
                                      ${isSelected ? 'bg-[#1C1917]' : 'bg-white'}`}>
                           <span className={`text-xs font-bold tracking-widest uppercase
                                          ${isSelected ? 'text-white' : 'text-[#1C1917]/40'}`}>
                              Capacity
                           </span>
                           <span className={`font-mono text-sm
                                          ${isSelected ? 'text-[#FAF9F6]' : 'text-[#1C1917]'}`}>
                              {total}/5
                           </span>
                        </div>
                      </button>
                    );
                  })}
                </div>
             </div>
          </div>

          {/* RIGHT: CONTROLS */}
          <div className="col-span-12 lg:col-span-5 flex flex-col gap-6">
            
            <div className="bg-[#FAF9F6] rounded-[2rem] p-8 shadow-sm flex-1">
              <div className="flex items-center justify-between mb-8">
                <div>
                   <h3 className="text-xl font-bold text-[#1C1917]">Well 0{selectedWell + 1}</h3>
                   <p className="text-[#1C1917]/40 text-sm">Material Composition</p>
                </div>
                {getCurrentTotal(selectedWell) === 5 && (
                  <div className="px-3 py-1 bg-[#1C1917] text-white rounded-full text-xs font-bold flex items-center gap-2">
                    <AlertCircle className="w-3 h-3" />
                    MAX
                  </div>
                )}
              </div>

              <div className="space-y-6">
                {materials.map((material) => (
                  <div key={material.key} className="bg-white p-4 rounded-xl border border-transparent hover:border-[#1C1917]/10 transition-colors">
                    <div className="flex justify-between items-center mb-4">
                      <div className="flex items-center gap-3">
                        <div 
                          className="w-3 h-3 rounded-full shadow-sm"
                          style={{ backgroundColor: material.color }}
                        />
                        <span className="font-bold text-[#1C1917]">{material.name}</span>
                      </div>
                      <span className="text-xs font-mono bg-[#EAE8E4] px-2 py-1 rounded text-[#1C1917]/60">
                        {configs[selectedWell][material.key]} / 3
                      </span>
                    </div>

                    <div className="flex items-center gap-4">
                       <button
                          onClick={() => handleDecrement(selectedWell, material.key)}
                          disabled={configs[selectedWell][material.key] === 0}
                          className="w-10 h-10 rounded-full bg-[#EAE8E4] border border-transparent flex items-center justify-center
                                   text-[#1C1917] hover:bg-[#1C1917] hover:text-white disabled:opacity-30 disabled:hover:bg-[#EAE8E4] disabled:hover:text-[#1C1917] transition-all"
                        >
                          <Minus className="w-4 h-4" />
                        </button>
                        
                        <div className="flex-1 h-2 bg-[#EAE8E4] rounded-full overflow-hidden">
                           <div 
                              className="h-full transition-all duration-300"
                              style={{ 
                                width: `${(configs[selectedWell][material.key] / 3) * 100}%`,
                                backgroundColor: material.color
                              }}
                            />
                        </div>

                        <button
                          onClick={() => handleIncrement(selectedWell, material.key)}
                          disabled={!canIncrement(selectedWell, material.key)}
                          className="w-10 h-10 rounded-full bg-[#1C1917] flex items-center justify-center
                                   text-white hover:bg-black disabled:opacity-30 disabled:bg-[#1C1917] transition-all shadow-md"
                        >
                          <Plus className="w-4 h-4" />
                        </button>
                    </div>
                  </div>
                ))}
              </div>
            </div>

            {/* ▼▼▼ [Logic] 버튼 연결: onComplete 대신 handleStartProduction 사용 ▼▼▼ */}
            <button
              onClick={handleStartProduction}
              disabled={!isValid}
              className="w-full py-6 bg-[#1C1917] text-white rounded-2xl font-bold tracking-[0.2em] uppercase
                       hover:bg-black transition-all disabled:opacity-50 disabled:cursor-not-allowed shadow-xl hover:-translate-y-1"
            >
              Start Production
            </button>

          </div>
        </div>
      </div>

      <RobotMonitor isOpen={showRobotMonitor} onClose={() => setShowRobotMonitor(false)} />
    </div>
  );
}