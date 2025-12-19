import React, { useState, useEffect } from 'react';
import { CheckCircle2, Loader2, Bot, GripHorizontal, Power, ShieldCheck, Zap } from 'lucide-react';

interface SystemCheckProps {
  onComplete: () => void;
}

export function SystemCheck({ onComplete }: SystemCheckProps) {
  const [activeStep, setActiveStep] = useState(0);
  const [progress, setProgress] = useState(0);

  const steps = [
    { 
      id: 1, 
      label: "INITIALIZING ROBOTICS", 
      sub: "Calibrating M0609 Joint Angles", 
      icon: Bot
    },
    { 
      id: 2, 
      label: "GRIPPER DIAGNOSTICS", 
      sub: "Testing Pneumatic Actuators", 
      icon: GripHorizontal
    },
    { 
      id: 3, 
      label: "SAFETY PROTOCOLS", 
      sub: "Verifying Interlock Systems", 
      icon: ShieldCheck
    },
    { 
      id: 4, 
      label: "SYSTEM READY", 
      sub: "Starting Beauro OS", 
      icon: Power
    }
  ];

  useEffect(() => {
    // Progress bar animation
    const progressInterval = setInterval(() => {
      setProgress(prev => Math.min(prev + 0.5, 100));
    }, 30);

    // Step transition
    const stepInterval = setInterval(() => {
      setActiveStep(prev => {
        if (prev < steps.length - 1) return prev + 1;
        clearInterval(stepInterval);
        return prev;
      });
    }, 1500);

    // Completion
    const timeout = setTimeout(() => {
      onComplete();
    }, 6000);

    return () => {
      clearInterval(progressInterval);
      clearInterval(stepInterval);
      clearTimeout(timeout);
    };
  }, [onComplete, steps.length]);

  return (
    <div className="min-h-screen bg-[#D8C3BE] flex flex-col items-center justify-center font-sans p-6">
      
      {/* Central Visualization */}
      <div className="relative w-full max-w-2xl bg-[#FAF9F6] rounded-[2rem] p-12 shadow-xl border border-white/20">
        
        <div className="relative z-10">
          {/* Header */}
          <div className="flex justify-between items-start mb-16">
            <div>
              <div className="flex items-center gap-2 mb-3">
                <Zap className="w-4 h-4 text-[#1C1917]" />
                <span className="text-xs font-bold tracking-[0.3em] text-[#1C1917]/50 uppercase">System Status</span>
              </div>
              <h2 className="text-3xl font-bold text-[#1C1917] tracking-tight">Diagnostic Sequence</h2>
            </div>
            <div className="text-right">
              <div className="text-5xl font-serif text-[#1C1917]">{Math.round(progress)}%</div>
            </div>
          </div>

          {/* Steps Visualization */}
          <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
            {steps.map((step, index) => {
              const Icon = step.icon;
              const isActive = index === activeStep;
              const isCompleted = index < activeStep;

              return (
                <div 
                  key={step.id}
                  className={`relative p-6 rounded-xl transition-all duration-500 overflow-hidden flex flex-col justify-between h-32 border
                    ${isActive 
                      ? 'bg-[#1C1917] text-[#FAF9F6] border-[#1C1917] shadow-lg scale-[1.02]' 
                      : isCompleted 
                        ? 'bg-[#EAE8E4] border-transparent opacity-60' 
                        : 'bg-transparent border-[#1C1917]/10 opacity-40'
                    }`}
                >
                  <div className="flex items-start justify-between">
                    <div className={`p-2 rounded-lg ${isActive ? 'bg-white/20' : 'bg-white/50'}`}>
                       <Icon className={`w-5 h-5 ${isActive ? 'text-white' : 'text-[#1C1917]'}`} />
                    </div>
                    {isActive && <Loader2 className="w-5 h-5 text-white/50 animate-spin" />}
                    {isCompleted && <CheckCircle2 className="w-5 h-5 text-[#1C1917]" />}
                  </div>

                  <div>
                    <h3 className={`font-bold text-xs uppercase tracking-widest mb-1 ${isActive ? 'text-white' : 'text-[#1C1917]'}`}>
                      {step.label}
                    </h3>
                    <p className={`text-[10px] ${isActive ? 'text-white/60' : 'text-[#1C1917]/50'}`}>
                      {step.sub}
                    </p>
                  </div>
                </div>
              );
            })}
          </div>

          {/* Bottom Progress Bar */}
          <div className="mt-12 h-1 bg-[#1C1917]/10 rounded-full overflow-hidden">
            <div 
              className="h-full bg-[#1C1917] transition-all duration-100 ease-out"
              style={{ width: `${progress}%` }}
            />
          </div>
          <div className="mt-4 flex justify-between text-[10px] font-bold text-[#1C1917]/40 uppercase tracking-widest">
             <span>Est. Time: 0:06</span>
             <span>System v2.4.0</span>
          </div>

        </div>
      </div>
    </div>
  );
}
