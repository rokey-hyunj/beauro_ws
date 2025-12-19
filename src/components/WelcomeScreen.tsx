import React from 'react';
import { ArrowRight } from 'lucide-react';
import heroImage from '../assets/1f3d288be679d3db9cae704e5e03f8aad9696f47.png';
import titleLogo from '../assets/09fba570a9059cf0fb2e7ca6a0d41fcd9bad7d56.png';

interface WelcomeScreenProps {
  onStart: () => void;
}

export function WelcomeScreen({ onStart }: WelcomeScreenProps) {
  return (
    <div className="relative min-h-screen flex flex-col items-center justify-center overflow-hidden font-sans">
      
      {/* Background Image Element */}
      <div className="absolute inset-0">
        <img 
          src={heroImage} 
          alt="Luxury Cosmetics" 
          className="w-full h-full object-cover"
        />
        {/* Dark overlay for contrast */}
        <div className="absolute inset-0 bg-black/50" />
      </div>

      {/* Main Content Container */}
      <div className="relative z-10 flex flex-col items-center justify-center w-full max-w-7xl px-6 animate-in fade-in duration-1000 slide-in-from-bottom-8">
        
        {/* Logo Section */}
        <div className="mb-12 flex flex-col items-center w-full">
          {/* Logo Image - Original Colors */}
          <h1 className="text-7xl md:text-9xl font-serif text-white tracking-[0.1em] mb-8 font-light drop-shadow-2xl">
            BEAURO
          </h1>
          
          {/* Subtitle Divider */}
          <div className="flex items-center gap-6 opacity-90">
            <div className="h-px w-16 bg-white/40" />
            <p className="text-sm md:text-base text-white tracking-[0.4em] uppercase font-medium">
              Robotic Cosmetics Formulation
            </p>
            <div className="h-px w-16 bg-white/40" />
          </div>
        </div>

        {/* Brand Philosophy Text */}
        <div className="text-center space-y-6 mb-16 max-w-2xl">
          <h2 className="text-3xl md:text-4xl text-white font-serif leading-snug drop-shadow-lg font-medium">
            기술과 미학의 완벽한 조화,<br />
            <span className="text-white/80 text-2xl md:text-3xl mt-2 block">당신만을 위한 정밀 맞춤형 솔루션</span>
          </h2>
        </div>

        {/* Action Area */}
        <button
          onClick={onStart}
          className="group relative px-12 py-5 bg-white/10 backdrop-blur-sm border border-white/20 text-white overflow-hidden 
                     hover:bg-white/20 transition-all duration-500 ease-out hover:-translate-y-0.5 rounded-sm flex items-center gap-4"
        >
          <span className="tracking-[0.2em] uppercase text-sm font-bold">Start Formulation</span>
          <ArrowRight className="w-4 h-4 group-hover:translate-x-1 transition-transform duration-300" />
        </button>

      </div>

      {/* Footer System Status */}
      <div className="absolute bottom-10 left-0 right-0 flex justify-center">
         <div className="flex items-center gap-2.5 px-4 py-2 bg-black/20 backdrop-blur-md rounded-full border border-white/5">
            <span className="relative flex h-2 w-2">
              <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-green-400 opacity-75"></span>
              <span className="relative inline-flex rounded-full h-2 w-2 bg-green-500"></span>
            </span>
            <span className="text-[10px] font-bold tracking-[0.2em] text-white/70 uppercase">System Operational</span>
         </div>
      </div>
    </div>
  );
}
