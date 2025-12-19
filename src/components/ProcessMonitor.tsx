// src/components/ProcessMonitor.tsx

import React, { useState, useEffect } from 'react';
import { Package, Droplet, Beaker, Activity, WifiOff } from 'lucide-react';
import type { ProductType, WellConfig } from '../types'; // types 경로 확인 필요
import { RobotMonitor } from './RobotMonitor';
import { MonitorButton } from './MonitorButton';

// ▼▼▼ [Logic] Firebase 라이브러리 추가 ▼▼▼
import { ref, onValue } from "firebase/database";
import { db } from "../../lib/firebase";

interface ProcessMonitorProps {
  wellConfigs: WellConfig[];
  productType: ProductType;
  onComplete: () => void;
}

export function ProcessMonitor({ wellConfigs, productType, onComplete }: ProcessMonitorProps) {
  // ▼▼▼ [Logic] 실제 로봇 상태 State ▼▼▼
  const [robotStatus, setRobotStatus] = useState({
    current_action: "Initializing...",
    progress: 0,
    current_well: 0,
    is_online: false,
    state: 0
  });

  const [showRobotMonitor, setShowRobotMonitor] = useState(false);

  // ▼▼▼ [Logic] Firebase 실시간 데이터 수신 ▼▼▼
  useEffect(() => {
    const statusRef = ref(db, 'robot_state');

    const unsubscribe = onValue(statusRef, (snapshot) => {
      const data = snapshot.val();
      if (data) {
        setRobotStatus({
          current_action: data.current_action || "Waiting for command...",
          progress: data.progress_percent || 0,
          current_well: data.current_well || 0,
          is_online: data.is_online || false,
          state: data.state || 0
        });

        // 진행률 100% 도달 시 완료 처리
        if (data.progress_percent >= 100) {
          setTimeout(() => {
            onComplete();
          }, 1500);
        }
      }
    });

    return () => unsubscribe();
  }, [onComplete]);

  // ▼▼▼ [UI Logic] 현재 동작에 따른 아이콘 결정 Helper ▼▼▼
  const getCurrentIcon = () => {
    const action = robotStatus.current_action.toLowerCase();
    if (action.includes('liquid')) return <Droplet className="w-10 h-10 text-[#1C1917]" />;
    if (action.includes('powder')) return <Beaker className="w-10 h-10 text-[#1C1917]" />;
    if (action.includes('tool') || action.includes('change')) return <Package className="w-10 h-10 text-[#1C1917]" />;
    if (action.includes('return')) return <Package className="w-10 h-10 text-[#1C1917]/40" />;
    return <Activity className="w-10 h-10 text-[#1C1917] animate-pulse" />;
  };

  // ▼▼▼ [UI Logic] 로봇 오프라인 상태 처리 ▼▼▼
  if (!robotStatus.is_online) {
    return (
      <div className="min-h-screen bg-[#D8C3BE] flex items-center justify-center font-sans">
        <div className="text-center flex flex-col items-center">
           <WifiOff className="w-12 h-12 text-[#1C1917]/50 mb-4" />
           <h2 className="text-2xl font-bold text-[#1C1917] tracking-tight uppercase">Robot Offline</h2>
           <p className="text-[#1C1917]/50 mt-2">Waiting for connection...</p>
           {/* 개발용 강제 이동 버튼 */}
           <button onClick={onComplete} className="mt-8 text-xs underline text-[#1C1917]/30 hover:text-[#1C1917]">
             [DEV] Skip to Next
           </button>
        </div>
      </div>
    );
  }

  return (
    <div className="min-h-screen bg-[#D8C3BE] p-6 md:p-12 font-sans flex flex-col">
      <div className="max-w-7xl mx-auto w-full flex-1 flex flex-col">
        {/* Header */}
        <div className="flex items-start justify-between mb-12">
          <div>
             <div className="flex items-center gap-3 mb-2">
                <span className={`w-2 h-2 rounded-full animate-pulse ${robotStatus.is_online ? 'bg-green-500' : 'bg-red-500'}`} />
                <span className="text-xs font-bold tracking-[0.2em] text-[#1C1917]/50 uppercase">Live Process</span>
             </div>
             <h1 className="text-3xl md:text-4xl font-bold text-[#1C1917] tracking-tight uppercase">
              Formulation
             </h1>
          </div>
          
           <MonitorButton onClick={() => setShowRobotMonitor(true)} />
        </div>

        <div className="flex-1 grid grid-cols-1 lg:grid-cols-2 gap-8 items-center">
          
          {/* LEFT: Current Action */}
          <div className="bg-[#FAF9F6] rounded-[2rem] p-12 shadow-sm relative overflow-hidden min-h-[500px] flex flex-col justify-center">
             {/* Background Decoration */}
             <div className="absolute top-0 right-0 p-12 opacity-[0.03]">
                <Package className="w-64 h-64" />
             </div>
             
             <div className="relative z-10">
                <div className="inline-flex items-center justify-center w-20 h-20 rounded-2xl bg-[#EAE8E4] mb-8 shadow-inner transition-all duration-300">
                   {getCurrentIcon()}
                </div>

                <div className="space-y-4 mb-12">
                   {/* 로봇이 보내주는 Action 텍스트 표시 */}
                   <h2 className="text-4xl md:text-5xl font-bold text-[#1C1917] tracking-tight leading-tight uppercase break-words">
                      {robotStatus.current_action}
                   </h2>
                   <p className="text-xl text-[#1C1917]/60 font-serif italic">
                      {robotStatus.state === 1 ? "Moving to Position..." : "Processing Command..."}
                   </p>
                </div>

                {/* Progress Bar */}
                <div className="space-y-2">
                   <div className="flex justify-between text-xs font-bold uppercase tracking-widest text-[#1C1917]/40">
                      <span>Step Progress</span>
                      <span>{robotStatus.progress}%</span>
                   </div>
                   <div className="h-2 w-full bg-[#EAE8E4] rounded-full overflow-hidden">
                      <div 
                        className="h-full bg-[#1C1917] transition-all duration-300 ease-linear"
                        style={{ width: `${robotStatus.progress}%` }}
                      />
                   </div>
                </div>
             </div>
          </div>

          {/* RIGHT: Status & Tray */}
          <div className="flex flex-col gap-8 h-full">
             {/* Overall Status Card */}
             <div className="bg-[#1C1917] rounded-[2rem] p-8 text-white relative overflow-hidden shadow-lg">
                <div className="relative z-10 flex justify-between items-end">
                   <div>
                      <h3 className="text-[#FAF9F6]/40 text-xs font-bold tracking-[0.2em] uppercase mb-2">Total Progress</h3>
                      <div className="text-4xl font-bold">{robotStatus.progress}%</div>
                   </div>
                   <Activity className="w-12 h-12 text-[#FAF9F6]/20" />
                </div>
                {/* Background Progress Fill */}
                <div 
                  className="absolute bottom-0 left-0 top-0 bg-[#FAF9F6]/30 transition-all duration-300 ease-out"
                  style={{ width: `${robotStatus.progress}%` }}
                />
             </div>

             {/* Tray Grid */}
             <div className="bg-[#FAF9F6] rounded-[2rem] p-8 shadow-sm flex-1 flex flex-col">
                <h3 className="text-[#1C1917]/40 text-xs font-bold tracking-[0.2em] uppercase mb-6 text-center">Tray Status</h3>
                
                <div className="flex-1 grid grid-cols-3 gap-4">
                  {wellConfigs.map((config, index) => {
                    // Firebase의 current_well (1~6)과 매칭
                    const isActive = robotStatus.current_well === (index + 1);
                    
                    return (
                      <div
                        key={index}
                        className={`rounded-xl flex flex-col items-center justify-center transition-all duration-300
                                  ${isActive 
                                    ? 'bg-white ring-2 ring-[#1C1917] shadow-lg scale-105' 
                                    : 'bg-[#EAE8E4] border border-transparent'}`}
                      >
                        <span className={`text-2xl font-bold font-serif mb-1 ${isActive ? 'text-[#1C1917]' : 'text-[#1C1917]/20'}`}>
                          0{index + 1}
                        </span>
                        {isActive && (
                           <span className="w-1.5 h-1.5 rounded-full bg-[#1C1917] animate-ping" />
                        )}
                      </div>
                    );
                  })}
                </div>
             </div>
          </div>
        </div>
      </div>
      <RobotMonitor isOpen={showRobotMonitor} onClose={() => setShowRobotMonitor(false)} />
    </div>
  );
}