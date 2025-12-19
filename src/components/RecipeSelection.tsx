import React, { useState } from 'react';
import { Droplet, Palette, ArrowRight } from 'lucide-react';
import type { ProductType } from '../types';
import { RobotMonitor } from './RobotMonitor';
import { MonitorButton } from './MonitorButton';
import foundationImage from '../assets/61237de1bed70869f76e26871a1d7619d70feb25.png';
import liptintImage from '../assets/ee2be5ba856ab9696641a6022081c4bd6ddf2afb.png';

interface RecipeSelectionProps {
  onSelect: (type: ProductType) => void;
}

export function RecipeSelection({ onSelect }: RecipeSelectionProps) {
  const [showRobotMonitor, setShowRobotMonitor] = useState(false);

  const products = [
    {
      type: 'foundation' as const,
      name: 'Foundation',
      nameKo: '파운데이션',
      icon: Droplet,
      description: 'Liquid base formulation',
      descriptionKo: '액상 베이스 포뮬레이션',
      image: foundationImage
    },
    {
      type: 'liptint' as const,
      name: 'Lip Tint',
      nameKo: '립틴트',
      icon: Palette,
      description: 'Color cosmetics blend',
      descriptionKo: '컬러 코스메틱 블렌드',
      image: liptintImage
    }
  ];

  return (
    <div className="min-h-screen bg-[#D8C3BE] p-6 md:p-12 font-sans flex flex-col">
      <div className="max-w-7xl mx-auto w-full flex-1 flex flex-col">
        {/* Header */}
        <div className="flex items-start justify-between mb-12">
          <div>
            <span className="block text-xs font-bold tracking-[0.3em] text-[#1C1917]/50 uppercase mb-2">Formulation Phase 01</span>
            <h1 className="text-4xl md:text-5xl font-bold text-[#1C1917] tracking-tight uppercase">
               Select Product
            </h1>
          </div>
          
          <MonitorButton onClick={() => setShowRobotMonitor(true)} />
        </div>

        {/* Product Cards */}
        <div className="flex-1 grid grid-cols-1 md:grid-cols-2 gap-8 items-center">
          {products.map((product) => {
            return (
              <button
                key={product.type}
                onClick={() => onSelect(product.type)}
                className="group relative h-[500px] w-full bg-[#FAF9F6] rounded-[2rem] shadow-sm
                         hover:shadow-xl transition-all duration-500 overflow-hidden text-left
                         flex flex-col border border-white/40"
              >
                {/* Abstract Background */}
                <div className="absolute inset-0 bg-white/0 group-hover:bg-[#1C1917]/5 transition-colors duration-500" />
                
                {/* Product Image Background */}
                <div className="absolute -right-20 -bottom-20 w-[600px] h-[600px] opacity-40 rotate-12 
                              group-hover:scale-105 group-hover:-rotate-6 transition-all duration-700 ease-out pointer-events-none mix-blend-multiply">
                  <img src={product.image} alt="" className="w-full h-full object-contain" />
                </div>
                
                <div className="relative z-10 flex-1 p-10 flex flex-col justify-between">
                  <div className="relative">
                    {/* Glass backdrop for text readability if image overlaps */}
                    <div className="absolute -inset-4 bg-[#FAF9F6]/60 blur-xl -z-10 rounded-full opacity-0 group-hover:opacity-100 transition-opacity duration-500" />
                    
                    <h3 className="text-4xl font-bold text-[#1C1917] mb-2 font-serif tracking-tight drop-shadow-sm">
                      {product.name}
                    </h3>
                    <p className="text-xl text-[#1C1917]/70 font-serif italic mb-6 font-medium">
                      {product.nameKo}
                    </p>
                    
                    <div className="space-y-1">
                      <p className="text-xs font-bold text-[#1C1917]/50 uppercase tracking-widest">Description</p>
                      <p className="text-[#1C1917] leading-relaxed font-medium">
                        {product.description}
                        <br />
                        <span className="text-[#1C1917]/50 text-sm font-normal">{product.descriptionKo}</span>
                      </p>
                    </div>
                  </div>

                  <div className="flex items-center justify-between border-t border-[#1C1917]/10 pt-8 mt-4 backdrop-blur-sm bg-white/30 -mx-10 px-10 pb-4 -mb-4">
                    <span className="text-xs font-bold text-[#1C1917]/60 tracking-[0.2em] uppercase group-hover:tracking-[0.25em] transition-all">
                      Configure Specs
                    </span>
                    <div className="w-10 h-10 rounded-full bg-[#EAE8E4]/80 flex items-center justify-center text-[#1C1917] group-hover:bg-[#1C1917] group-hover:text-white transition-all duration-300 shadow-sm">
                      <ArrowRight className="w-4 h-4" />
                    </div>
                  </div>
                </div>
              </button>
            );
          })}
        </div>
      </div>

      <RobotMonitor isOpen={showRobotMonitor} onClose={() => setShowRobotMonitor(false)} />
    </div>
  );
}
