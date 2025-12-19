// src/components/MixingScreen.tsx

import React, { useState, useEffect } from 'react';
import { Waves, Activity, Check, Package, WifiOff } from 'lucide-react';
import type { WellConfig } from '../types'; // types 경로 확인
import { RobotMonitor } from './RobotMonitor';
import { MonitorButton } from './MonitorButton';

// ▼▼▼ [Logic] Firebase 라이브러리 추가 ▼▼▼
import { ref, onValue } from "firebase/database";
import { db } from "../../lib/firebase";

interface MixingScreenProps {
  wellConfigs: WellConfig[];
  onComplete: () => void;
}

export function MixingScreen({ wellConfigs, onComplete }: MixingScreenProps) {
  // ▼▼▼ [Logic] 실제 로봇 상태 State ▼▼▼
  const [robotStatus, setRobotStatus] = useState({
    current_action: "Initializing Mixer...",
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
          current_action: data.current_action || "Waiting...",
          progress: data.progress_percent || 0,
          current_well: data.current_well || 0,
          is_online: data.is_online || false,
          state: data.state || 0
        });

        // [Logic] 배출(Output) 단계 완료 시 리포트 화면으로 이동
        // 로봇이 "Output" 작업을 마치고 진행률 100%가 되면 이동
        if (data.current_action.includes("Output") && data.progress_percent >= 100) {
           setTimeout(() => onComplete(), 2000);
        }
      }
    });

    return () => unsubscribe();
  }, [onComplete]);

  // ▼▼▼ [UI Logic] 배출 단계인지 확인 (화면 전환용) ▼▼▼
  const isOutputPhase = robotStatus.current_action.includes("Output") || robotStatus.current_action.includes("Transfer");

  // 1. 배출(Output) 단계 화면
  if (isOutputPhase) {
    return (
      <div className="min-h-screen bg-[#D8C3BE] flex items-center justify-center font-sans">
        <div className="text-center animate-pulse">
           <Package className="w-16 h-16 mx-auto mb-4 text-[#1C1917]" />
           <h2 className="text-2xl font-bold text-[#1C1917] tracking-tight uppercase">Batch Complete</h2>
           <p className="text-[#1C1917]/50 mt-2">Transferring to Output Zone...</p>
        </div>
      </div>
    );
  }

  // 2. 오프라인 화면
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

  // 3. 메인 교반(Mixing) 화면
  const overallProgress = (robotStatus.current_well / 6) * 100;

  return (
    <div className="min-h-screen bg-[#D8C3BE] p-6 md:p-12 font-sans flex flex-col">
      <div className="max-w-7xl mx-auto w-full flex-1 flex flex-col">
        {/* Header */}
        <div className="flex items-start justify-between mb-12">
          <div>
             <div className="flex items-center gap-3 mb-2">
                <span className="w-2 h-2 rounded-full bg-[#1C1917] animate-pulse" />
                <span className="text-xs font-bold tracking-[0.2em] text-[#1C1917]/50 uppercase">Mixing Phase</span>
             </div>
             <h1 className="text-3xl md:text-4xl font-bold text-[#1C1917] tracking-tight uppercase">
              Homogenization
             </h1>
          </div>
          
           <MonitorButton onClick={() => setShowRobotMonitor(true)} />
        </div>

        <div className="flex-1 grid grid-cols-1 lg:grid-cols-2 gap-8 items-center">
          
          {/* LEFT: Mixing Visualizer */}
          <div className="bg-[#FAF9F6] rounded-[2rem] p-12 shadow-sm relative overflow-hidden min-h-[500px] flex flex-col justify-center text-center">
             
             {/* Center Animated Icon */}
             <div className="flex justify-center mb-10 relative">
                <div className="relative z-10 w-32 h-32 rounded-full bg-[#EAE8E4] flex items-center justify-center shadow-inner">
                   {/* 로봇이 작업 중일 때만 애니메이션 작동 */}
                   <Waves className={`w-12 h-12 text-[#1C1917] ${robotStatus.progress > 0 ? 'animate-pulse' : ''}`} />
                </div>
                {/* Ripples (작업 중일 때만 표시) */}
                {robotStatus.progress > 0 && (
                   <>
                      <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-48 h-48 bg-[#1C1917]/10 rounded-full animate-ping opacity-20" />
                      <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-64 h-64 bg-[#1C1917]/5 rounded-full animate-ping delay-100 opacity-20" />
                   </>
                )}
             </div>

             <div className="space-y-4 mb-12 relative z-10">
                <p className="text-xs font-bold text-[#1C1917]/40 tracking-[0.3em] uppercase">Current Operation</p>
                <h2 className="text-4xl md:text-5xl font-bold text-[#1C1917] tracking-tight uppercase break-words">
                   {/* 로봇이 보내주는 텍스트 표시 (예: Mixing Well 1...) */}
                   {robotStatus.current_action}
                </h2>
                <p className="text-xl text-[#1C1917]/60 font-serif italic">
                   {robotStatus.state === 1 ? "Moving to position..." : "Spiral Mixing Pattern"}
                </p>
             </div>

             {/* Circular Progress Indicator for current well */}
             <div className="mx-auto w-16 h-1 rounded-full bg-[#EAE8E4] overflow-hidden">
                <div 
                  className="h-full bg-[#1C1917] transition-all duration-300"
                  style={{ width: `${robotStatus.progress}%` }}
                />
             </div>
             <p className="mt-2 text-xs font-mono text-[#1C1917]/40">
                Step Progress: {robotStatus.progress}%
             </p>
          </div>

          {/* RIGHT: Grid Status */}
          <div className="flex flex-col gap-8 h-full">
             
             {/* Overall Progress */}
             <div className="bg-[#1C1917] rounded-[2rem] p-8 text-white flex items-center justify-between shadow-lg">
                <div>
                   <h3 className="text-[#FAF9F6]/40 text-xs font-bold tracking-[0.2em] uppercase mb-1">Batch Progress</h3>
                   <div className="text-2xl font-bold">
                      {/* 전체 공정률 계산 (웰 기준) */}
                      {Math.min(100, Math.round(overallProgress))}% Complete
                   </div>
                </div>
                <div className="w-12 h-12 rounded-full border-2 border-white/20 flex items-center justify-center">
                   <Activity className="w-6 h-6 text-white" />
                </div>
             </div>

             {/* Tray Grid Visualization */}
             <div className="bg-[#FAF9F6] rounded-[2rem] p-8 shadow-sm flex-1">
                <div className="grid grid-cols-3 gap-6 h-full">
                  {wellConfigs.map((config, index) => {
                    // Firebase current_well (1~6)과 매칭
                    const isCurrent = (index + 1) === robotStatus.current_well;
                    const isComplete = (index + 1) < robotStatus.current_well;
                    
                    return (
                      <div
                        key={index}
                        className={`rounded-2xl flex flex-col items-center justify-center transition-all duration-500
                                  ${isCurrent 
                                    ? 'bg-white ring-2 ring-[#1C1917] scale-105 shadow-md' 
                                    : isComplete
                                       ? 'bg-[#1C1917] text-white shadow-md'
                                       : 'bg-[#EAE8E4] opacity-50'}`}
                      >
                        <span className={`text-2xl font-bold font-serif mb-2 ${isCurrent ? 'text-[#1C1917]' : isComplete ? 'text-white' : 'text-[#1C1917]/20'}`}>
                          0{index + 1}
                        </span>
                        
                        {isComplete ? (
                           <Check className="w-5 h-5 text-white" />
                        ) : isCurrent ? (
                           <span className="text-[10px] font-bold tracking-widest text-[#1C1917]/40 uppercase animate-pulse">Mixing</span>
                        ) : (
                           <span className="text-[10px] font-bold tracking-widest text-[#1C1917]/20 uppercase">Pending</span>
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