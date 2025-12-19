import React from 'react';
import { Activity } from 'lucide-react';

interface MonitorButtonProps {
  onClick: () => void;
  className?: string;
}

export function MonitorButton({ onClick, className = '' }: MonitorButtonProps) {
  return (
    <button
      onClick={onClick}
      className={`group relative flex items-center gap-3 pl-5 pr-6 py-3 bg-[#FAF9F6] border border-white/50 
                 rounded-xl shadow-lg hover:shadow-xl hover:border-[#1C1917]/20 transition-all duration-300 
                 hover:-translate-y-0.5 overflow-hidden ${className}`}
    >
      <div className="absolute inset-0 bg-[#EAE8E4] opacity-0 group-hover:opacity-100 transition-opacity" />
      
      <div className="relative flex items-center justify-center w-8 h-8 rounded-lg bg-[#1C1917] text-white group-hover:bg-[#3E2723] transition-colors duration-300">
        <Activity className="w-5 h-5" />
      </div>
      
      <div className="relative text-left">
        <div className="text-[10px] font-bold tracking-widest text-[#1C1917]/40 uppercase group-hover:text-[#1C1917] transition-colors">
          System
        </div>
        <div className="text-sm font-bold text-[#1C1917] leading-none group-hover:text-[#1C1917]">
          Monitor
        </div>
      </div>
    </button>
  );
}
