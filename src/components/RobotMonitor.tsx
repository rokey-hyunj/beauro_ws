import React, { useState, useEffect } from 'react';
import { Activity, X, Zap, Move3d, Axis3d, WifiOff, Lock, Unlock, Send, Home, AlertTriangle, PlayCircle, StopCircle, Pause, Play } from 'lucide-react';

import { ref, onValue, set, update } from "firebase/database";
import { db } from "../../lib/firebase";

interface RobotMonitorProps {
  isOpen: boolean;
  onClose: () => void;
}

interface RobotState {
  status: string;
  mode: string;
  is_online: boolean;
  is_error: boolean;
  error_message: string;
}

interface RobotTCP {
  x: number; y: number; z: number;
  a: number; b: number; c: number;
}

interface RobotJoints {
  j1: number; j2: number; j3: number; j4: number; j5: number; j6: number;
}

const HOME_POSE: RobotJoints = { j1: 0, j2: 0, j3: 90, j4: 0, j5: 90, j6: 0 };

export function RobotMonitor({ isOpen, onClose }: RobotMonitorProps) {
  const [robotState, setRobotState] = useState<RobotState>({
    status: 'OFFLINE', mode: 'UNKNOWN', is_online: false, is_error: false, error_message: ''
  });
  
  const [actualTcp, setActualTcp] = useState<RobotTCP>({ x: 0, y: 0, z: 0, a: 0, b: 0, c: 0 });
  const [actualJoints, setActualJoints] = useState<RobotJoints>({ j1: 0, j2: 0, j3: 0, j4: 0, j5: 0, j6: 0 });

  const [isManualMode, setIsManualMode] = useState(false);
  const [isPaused, setIsPaused] = useState(false); // 버튼 UI용 상태
  
  const [targetTcp, setTargetTcp] = useState<RobotTCP>({ x: 0, y: 0, z: 0, a: 0, b: 0, c: 0 });
  const [targetJoints, setTargetJoints] = useState<RobotJoints>({ j1: 0, j2: 0, j3: 0, j4: 0, j5: 0, j6: 0 });

  // 1. [기존] 로봇 상태 수신 (TCP, Joint, Status)
  useEffect(() => {
    if (!isOpen) return;
    const stateRef = ref(db, 'robot_state');
    const unsubscribe = onValue(stateRef, (snapshot) => {
      const data = snapshot.val();
      if (data) {
        let statusStr = 'UNKNOWN';
        if (data.state === 1) statusStr = 'IDLE';
        else if (data.state === 2) statusStr = 'WORKING';
        else if (data.state === 3) statusStr = 'ERROR';
        else if (data.state === 4) statusStr = 'PAUSED';
        else if (data.state === 0) statusStr = 'OFFLINE';

        let modeStr = data.mode === 1 ? 'AUTONOMOUS' : 'MANUAL';
        
        setRobotState({
          status: statusStr,
          mode: modeStr,
          is_online: data.is_online || false,
          is_error: data.is_error || false,
          error_message: data.error_message || ''
        });

        // [삭제됨] 여기서 setIsPaused를 하면 로봇 반응 속도 차이로 인해 버튼이 튀는 현상 발생
        // setIsPaused(data.state === 4); <--- 삭제

        if (data.tcp) {
          const newTcp = {
            x: data.tcp.x || 0, y: data.tcp.y || 0, z: data.tcp.z || 0,
            a: data.tcp.rx || 0, b: data.tcp.ry || 0, c: data.tcp.rz || 0
          };
          setActualTcp(newTcp);
          if (!isManualMode) setTargetTcp(newTcp);
        }

        if (data.joint && Array.isArray(data.joint)) {
          const newJoints = {
            j1: data.joint[0] || 0, j2: data.joint[1] || 0, j3: data.joint[2] || 0,
            j4: data.joint[3] || 0, j5: data.joint[4] || 0, j6: data.joint[5] || 0
          };
          setActualJoints(newJoints);
          if (!isManualMode) setTargetJoints(newJoints);
        }
      }
    });
    return () => unsubscribe();
  }, [isOpen, isManualMode]);

  // 2. [추가] 일시정지 명령 상태 수신 (버튼 동기화용)
  useEffect(() => {
    if (!isOpen) return;
    const commandRef = ref(db, 'command/is_paused');
    const unsubscribe = onValue(commandRef, (snapshot) => {
        // DB에 있는 명령 값(true/false)을 그대로 버튼 상태로 씀
        const pausedCmd = snapshot.val();
        setIsPaused(!!pausedCmd);
    });
    return () => unsubscribe();
  }, [isOpen]);


  const sendCommand = (type: 'tcp' | 'joint') => {
    if (!window.confirm(`⚠️ ${type.toUpperCase()} 이동 명령을 전송하시겠습니까? 로봇이 움직입니다.`)) return;

    const commandData = {
      type: type === 'tcp' ? 'move_line' : 'move_joint',
      target: type === 'tcp' ? targetTcp : [targetJoints.j1, targetJoints.j2, targetJoints.j3, targetJoints.j4, targetJoints.j5, targetJoints.j6],
      timestamp: Date.now()
    };

    set(ref(db, 'manual_command'), commandData)
      .then(() => alert("명령 전송 완료!"))
      .catch((err) => alert("전송 실패: " + err));
  };

  const handleGoHome = () => {
    if (!window.confirm("🏠 로봇을 초기 위치(HOME)로 이동시키겠습니까?")) return;
    setTargetJoints(HOME_POSE);
    const commandData = {
      type: 'move_joint',
      target: [HOME_POSE.j1, HOME_POSE.j2, HOME_POSE.j3, HOME_POSE.j4, HOME_POSE.j5, HOME_POSE.j6],
      timestamp: Date.now()
    };
    set(ref(db, 'manual_command'), commandData)
      .then(() => alert("HOME 이동 명령 전송!"))
      .catch((err) => alert("전송 실패: " + err));
  };

  const handleRecovery = (action: 'resume' | 'stop') => {
    if (!window.confirm(action === 'resume' ? "작업을 재개하시겠습니까?" : "작업을 완전히 중단하시겠습니까?")) return;
    set(ref(db, 'command/recovery'), action)
      .then(() => console.log(`Recovery command sent: ${action}`))
      .catch((err) => alert("명령 전송 실패: " + err));
  };

  // 일시정지 토글 핸들러
  const togglePause = () => {
    // 현재 상태의 반대로 명령 전송
    const nextState = !isPaused;
    set(ref(db, 'command/is_paused'), nextState)
      .catch((err) => alert("일시정지 명령 실패: " + err));
    // 여기서 setIsPaused를 직접 하지 않아도 위 useEffect가 감지하여 업데이트함
  };

  const handleTcpChange = (key: keyof RobotTCP, value: string) => {
    setTargetTcp(prev => ({ ...prev, [key]: parseFloat(value) || 0 }));
  };
  
  const handleJointChange = (key: keyof RobotJoints, value: string) => {
    setTargetJoints(prev => ({ ...prev, [key]: parseFloat(value) || 0 }));
  };

  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center p-4 bg-black/60 backdrop-blur-md animate-in fade-in duration-200">
      <div className="bg-[#FAF9F6] rounded-xl shadow-2xl w-full max-w-5xl max-h-[90vh] overflow-y-auto border border-white/20 relative">
        
        {/* 에러 팝업 */}
        {robotState.is_error && (
          <div className="absolute inset-0 z-50 bg-red-500/10 backdrop-blur-sm flex items-center justify-center p-4">
            <div className="bg-white p-8 rounded-2xl shadow-2xl max-w-md w-full border-2 border-red-500 animate-in zoom-in duration-300">
              <div className="flex flex-col items-center text-center">
                <div className="w-16 h-16 bg-red-100 rounded-full flex items-center justify-center mb-4">
                  <AlertTriangle className="w-8 h-8 text-red-600 animate-pulse" />
                </div>
                <h3 className="text-2xl font-bold text-red-600 mb-2">Robot Error Detected</h3>
                <p className="text-stone-600 mb-8 font-medium">
                  {robotState.error_message || "Unknown error occurred during operation."}
                </p>
                <div className="grid grid-cols-2 gap-4 w-full">
                  <button onClick={() => handleRecovery('stop')} className="flex items-center justify-center gap-2 px-6 py-4 bg-stone-100 text-stone-600 rounded-xl font-bold hover:bg-stone-200 transition-colors">
                    <StopCircle className="w-5 h-5" /> STOP
                  </button>
                  <button onClick={() => handleRecovery('resume')} className="flex items-center justify-center gap-2 px-6 py-4 bg-red-600 text-white rounded-xl font-bold hover:bg-red-700 transition-colors shadow-lg hover:shadow-xl">
                    <PlayCircle className="w-5 h-5" /> RESUME
                  </button>
                </div>
              </div>
            </div>
          </div>
        )}

        {/* Header */}
        <div className="flex items-center justify-between p-6 border-b border-[#1C1917]/10 bg-[#FAF9F6] sticky top-0 z-10">
          <div className="flex items-center gap-4">
            <h2 className="text-xl text-[#1C1917] font-serif flex items-center gap-3">
              <Activity className="w-6 h-6 text-[#1C1917]" />
              Robot Control Panel
            </h2>
            
            {/* 일시정지 버튼 */}
            {robotState.status !== 'IDLE' && robotState.status !== 'OFFLINE' && !robotState.is_error && (
                <button
                  onClick={togglePause}
                  className={`flex items-center gap-2 px-4 py-2 rounded-full text-xs font-bold tracking-widest transition-all shadow-sm
                    ${isPaused 
                      ? 'bg-green-100 text-green-700 hover:bg-green-200 ring-1 ring-green-300' 
                      : 'bg-yellow-100 text-yellow-700 hover:bg-yellow-200 ring-1 ring-yellow-300'}`}
                >
                  {isPaused ? (
                    <><Play className="w-3 h-3 fill-current" /> RESUME</>
                  ) : (
                    <><Pause className="w-3 h-3 fill-current" /> PAUSE</>
                  )}
                </button>
            )}

            <button
              onClick={() => setIsManualMode(!isManualMode)}
              className={`flex items-center gap-2 px-4 py-2 rounded-full text-xs font-bold tracking-widest transition-all
                ${isManualMode 
                  ? 'bg-red-500 text-white shadow-lg ring-2 ring-red-200' 
                  : 'bg-[#EAE8E4] text-[#1C1917]/50 hover:bg-[#1C1917]/10'}`}
            >
              {isManualMode ? <Unlock className="w-3 h-3" /> : <Lock className="w-3 h-3" />}
              {isManualMode ? "MANUAL ACTIVE" : "READ ONLY"}
            </button>
          </div>

          <div className="flex items-center gap-4">
            <div className="flex items-center gap-2">
              <div className={`w-2 h-2 rounded-full animate-pulse 
                  ${robotState.is_error ? 'bg-red-600' : 
                    isPaused ? 'bg-yellow-500' : // 버튼이 Pause 상태면 노란불 (UX 통일)
                    robotState.is_online ? 'bg-green-500' : 'bg-gray-400'}`} 
              />
              <span className="text-xs font-bold tracking-widest text-[#1C1917]/50">
                {robotState.is_online ? 'ONLINE' : 'OFFLINE'}
              </span>
            </div>
            <button onClick={onClose} className="p-2 hover:bg-[#1C1917]/10 rounded-full transition-colors text-[#1C1917]">
              <X className="w-5 h-5" />
            </button>
          </div>
        </div>

        {/* Content */}
        <div className="p-8 bg-[#FAF9F6] space-y-6">
          {!robotState.is_online && (
             <div className="p-4 bg-red-50 border border-red-100 rounded-lg flex items-center gap-3 text-red-800">
                <WifiOff className="w-5 h-5" />
                <span className="text-sm font-medium">Robot disconnected. Controls disabled.</span>
             </div>
          )}

          {/* PAUSED 알림 배너 */}
          {isPaused && (
             <div className="p-4 bg-yellow-50 border border-yellow-200 rounded-lg flex items-center justify-center gap-3 text-yellow-800 animate-pulse">
                <Pause className="w-5 h-5 fill-current" />
                <span className="text-sm font-bold uppercase tracking-wide">Operation Paused by User</span>
             </div>
          )}

          {/* Status Bar */}
          <div className={`p-6 rounded-lg border flex items-center justify-between shadow-sm transition-colors 
              ${robotState.is_error ? 'bg-red-50 border-red-200' : 
                isPaused ? 'bg-yellow-50 border-yellow-100' : 
                'bg-white border-stone-100'}`}>
             <div>
                <div className="text-xs font-bold text-[#1C1917]/40 tracking-widest mb-1">ROBOT STATUS</div>
                <div className="flex items-center gap-3">
                  <span className={`px-3 py-1 text-white text-xs font-bold tracking-wider rounded-sm 
                      ${robotState.is_error ? 'bg-red-600 animate-pulse' : 
                        isPaused ? 'bg-yellow-500' : // 명령 상태 우선 표시
                        robotState.is_online ? 'bg-[#1C1917]' : 'bg-gray-400'}`}>
                    {robotState.is_error ? 'ERROR' : isPaused ? 'PAUSED' : robotState.status}
                  </span>
                  <span className="text-[#1C1917] text-sm font-medium tracking-wide">{robotState.mode}</span>
                </div>
             </div>
             {robotState.is_error ? <AlertTriangle className="w-8 h-8 text-red-500 animate-bounce" /> : 
              isPaused ? <Pause className="w-8 h-8 text-yellow-500" /> :
              <Zap className={`w-8 h-8 ${isManualMode ? 'text-red-500' : 'text-stone-200'}`} />}
          </div>

          <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
            {/* TCP Control */}
            <div className={`p-6 rounded-lg border transition-all ${isManualMode ? 'bg-white border-red-200 ring-1 ring-red-100' : 'bg-white border-stone-100'}`}>
              <div className="flex items-center justify-between mb-4">
                <div className="flex items-center gap-2">
                  <Move3d className={`w-4 h-4 ${isManualMode ? 'text-red-500' : 'text-[#1C1917]/40'}`} />
                  <h3 className="text-sm font-bold text-[#1C1917] tracking-wide">TCP Control</h3>
                </div>
                {isManualMode && (
                  <button onClick={() => sendCommand('tcp')} className="flex items-center gap-2 px-3 py-1 bg-red-500 text-white text-xs font-bold rounded hover:bg-red-600 transition-colors">
                    <Send className="w-3 h-3" /> MOVE
                  </button>
                )}
              </div>
              <div className="space-y-3 font-mono text-sm">
                {Object.keys(targetTcp).map((key) => (
                  <div key={key} className="flex justify-between items-center border-b border-[#1C1917]/5 pb-2 last:border-0">
                    <span className="text-[#1C1917]/40 uppercase w-8">{key}</span>
                    {isManualMode ? (
                      <input type="number" value={targetTcp[key as keyof RobotTCP]} onChange={(e) => handleTcpChange(key as keyof RobotTCP, e.target.value)} className="w-24 text-right bg-stone-50 border border-stone-200 rounded px-2 py-1 text-[#1C1917] focus:outline-none focus:border-red-400" />
                    ) : (
                      <span className="text-[#1C1917]">{actualTcp[key as keyof RobotTCP].toFixed(2)}</span>
                    )}
                    <span className="text-[10px] text-[#1C1917]/40 w-8 text-right">{['x','y','z'].includes(key) ? 'mm' : 'deg'}</span>
                  </div>
                ))}
              </div>
            </div>

            {/* Joint Control */}
            <div className={`p-6 rounded-lg border transition-all ${isManualMode ? 'bg-white border-red-200 ring-1 ring-red-100' : 'bg-white border-stone-100'}`}>
              <div className="flex items-center justify-between mb-4">
                <div className="flex items-center gap-2">
                  <Axis3d className={`w-4 h-4 ${isManualMode ? 'text-red-500' : 'text-[#1C1917]/40'}`} />
                  <h3 className="text-sm font-bold text-[#1C1917] tracking-wide">Joint Control</h3>
                </div>
                {isManualMode && (
                  <div className="flex gap-2">
                    <button onClick={handleGoHome} className="flex items-center gap-2 px-3 py-1 bg-stone-600 text-white text-xs font-bold rounded hover:bg-stone-800 transition-colors" title="Move to Home Position">
                      <Home className="w-3 h-3" /> HOME
                    </button>
                    <button onClick={() => sendCommand('joint')} className="flex items-center gap-2 px-3 py-1 bg-red-500 text-white text-xs font-bold rounded hover:bg-red-600 transition-colors">
                      <Send className="w-3 h-3" /> MOVE
                    </button>
                  </div>
                )}
              </div>
              <div className="space-y-2 font-mono text-sm">
                {Object.keys(targetJoints).map((key) => (
                  <div key={key} className="flex justify-between items-center py-1 border-b border-[#1C1917]/5 last:border-0">
                    <span className="text-[#1C1917]/40 uppercase w-8">{key}</span>
                    {isManualMode ? (
                      <input type="number" value={targetJoints[key as keyof RobotJoints]} onChange={(e) => handleJointChange(key as keyof RobotJoints, e.target.value)} className="w-24 text-right bg-stone-50 border border-stone-200 rounded px-2 py-1 text-[#1C1917] focus:outline-none focus:border-red-400" />
                    ) : (
                      <span className="text-[#1C1917]">{actualJoints[key as keyof RobotJoints].toFixed(3)}</span>
                    )}
                    <span className="text-[10px] text-[#1C1917]/40 w-8 text-right">deg</span>
                  </div>
                ))}
              </div>
            </div>
          </div>
          
          {isManualMode && (
            <div className="text-center text-xs text-red-400 mt-4 animate-pulse">
              ⚠️ Warning: Manual control overrides safety checks. Ensure the robot path is clear.
            </div>
          )}
        </div>
      </div>
    </div>
  );
}