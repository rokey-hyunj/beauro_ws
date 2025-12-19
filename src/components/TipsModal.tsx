import React from 'react';
import { X, Lightbulb } from 'lucide-react';

interface TipStep {
  title: string;
  content: string;
}

interface TipsModalProps {
  tips: {
    title: string;
    steps: TipStep[];
  };
  onClose: () => void;
}

export function TipsModal({ tips, onClose }: TipsModalProps) {
  return (
    <div 
      className="fixed inset-0 z-50 flex items-center justify-center p-4 bg-black/60 backdrop-blur-sm"
      onClick={onClose}
    >
      <div 
        className="bg-white rounded-sm shadow-2xl max-w-2xl w-full max-h-[80vh] overflow-y-auto"
        onClick={(e) => e.stopPropagation()}
      >
        {/* Header */}
        <div className="sticky top-0 bg-white border-b border-[#e8e4dd] p-6 flex items-center justify-between">
          <div className="flex items-center space-x-3">
            <div className="p-2 bg-[#d4af37]/10 rounded-full">
              <Lightbulb className="w-6 h-6 text-[#d4af37]" />
            </div>
            <h2 className="text-xl tracking-wider text-[#3d3d3d]">{tips.title}</h2>
          </div>
          <button
            onClick={onClose}
            className="p-2 hover:bg-[#f5f2ed] rounded-full transition-colors"
          >
            <X className="w-6 h-6 text-[#8b7355]" />
          </button>
        </div>

        {/* Content */}
        <div className="p-6 space-y-6">
          {tips.steps.map((step, index) => (
            <div key={index} className="border-l-4 border-[#d4af37] pl-6 py-2">
              <h3 className="text-sm tracking-wider text-[#d4af37] mb-2">
                {step.title}
              </h3>
              <p className="text-[#3d3d3d] leading-relaxed">
                {step.content}
              </p>
            </div>
          ))}
        </div>

        {/* Footer */}
        <div className="sticky bottom-0 bg-[#f5f2ed] p-4 text-center">
          <button
            onClick={onClose}
            className="px-8 py-3 bg-[#d4af37] text-white hover:bg-[#c49d2f] 
                     transition-colors tracking-wider"
          >
            CLOSE
          </button>
        </div>
      </div>
    </div>
  );
}
