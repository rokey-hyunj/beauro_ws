import React, { useState } from 'react';
import { WelcomeScreen } from './components/WelcomeScreen';
import { SystemCheck } from './components/SystemCheck';
import { RecipeSelection } from './components/RecipeSelection';
import { MatrixSetup } from './components/MatrixSetup';
import { ProcessMonitor } from './components/ProcessMonitor';
import { MixingScreen } from './components/MixingScreen';
import { FinalReport } from './components/FinalReport';
import { ChevronLeft, ChevronRight } from 'lucide-react';
import type { ProductType, WellConfig } from './types';

export type ProcessStep = 'welcome' | 'system-check' | 'recipe' | 'matrix' | 'process' | 'mixing' | 'report';

function App() {
  const [currentStep, setCurrentStep] = useState<ProcessStep>('welcome');
  const [selectedProduct, setSelectedProduct] = useState<ProductType>(null);
  const [wellConfigs, setWellConfigs] = useState<WellConfig[]>(
    Array(6).fill(null).map(() => ({
      powder1: 0,
      powder2: 0,
      liquid1: 0,
      liquid2: 0
    }))
  );

  const steps: ProcessStep[] = ['welcome', 'system-check', 'recipe', 'matrix', 'process', 'mixing', 'report'];

  const nextStep = () => {
    const currentIndex = steps.indexOf(currentStep);
    if (currentIndex < steps.length - 1) {
      setCurrentStep(steps[currentIndex + 1]);
    }
  };

  const prevStep = () => {
    const currentIndex = steps.indexOf(currentStep);
    if (currentIndex > 0) {
      setCurrentStep(steps[currentIndex - 1]);
    }
  };

  const handleStart = () => {
    setCurrentStep('system-check');
  };

  const handleSystemReady = () => {
    setCurrentStep('recipe');
  };

  const handleRecipeSelect = (type: ProductType) => {
    setSelectedProduct(type);
    setCurrentStep('matrix');
  };

  const handleMatrixComplete = (configs: WellConfig[]) => {
    setWellConfigs(configs);
    setCurrentStep('process');
  };

  const handleProcessComplete = () => {
    setCurrentStep('mixing');
  };

  const handleMixingComplete = () => {
    setCurrentStep('report');
  };

  const handleRestart = () => {
    setCurrentStep('welcome');
    setSelectedProduct(null);
    setWellConfigs(Array(6).fill(null).map(() => ({
      powder1: 0,
      powder2: 0,
      liquid1: 0,
      liquid2: 0
    })));
  };

  return (
    <div className="min-h-screen bg-[#D8C3BE] relative">
      {currentStep === 'welcome' && <WelcomeScreen onStart={handleStart} />}
      {currentStep === 'system-check' && <SystemCheck onComplete={handleSystemReady} />}
      {currentStep === 'recipe' && <RecipeSelection onSelect={handleRecipeSelect} />}
      {currentStep === 'matrix' && (
        <MatrixSetup 
          productType={selectedProduct}
          initialConfigs={wellConfigs}
          onComplete={handleMatrixComplete}
          onBack={() => setCurrentStep('recipe')}
        />
      )}
      {currentStep === 'process' && (
        <ProcessMonitor
          wellConfigs={wellConfigs}
          productType={selectedProduct}
          onComplete={handleProcessComplete}
        />
      )}
      {currentStep === 'mixing' && (
        <MixingScreen
          wellConfigs={wellConfigs}
          onComplete={handleMixingComplete}
        />
      )}
      {currentStep === 'report' && (
        <FinalReport
          productType={selectedProduct}
          wellConfigs={wellConfigs}
          onRestart={handleRestart}
        />
      )}

      {/* Debug Navigation */}
      <div className="fixed bottom-4 right-4 flex gap-2 z-50 opacity-20 hover:opacity-100 transition-opacity">
        <button 
          onClick={prevStep}
          className="p-2 bg-black text-white rounded-full hover:bg-stone-700"
          title="Previous Step"
        >
          <ChevronLeft className="w-4 h-4" />
        </button>
        <button 
          onClick={nextStep}
          className="p-2 bg-black text-white rounded-full hover:bg-stone-700"
          title="Next Step"
        >
          <ChevronRight className="w-4 h-4" />
        </button>
      </div>
    </div>
  );
}

export default App;
