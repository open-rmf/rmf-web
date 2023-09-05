import React from 'react';
import { AppEvents } from './app-events';

interface CanvasControllerProps {
  children: React.ReactNode;
  disabledLayers: Record<string, boolean>;
}

export const CanvasController = ({ disabledLayers, children }: CanvasControllerProps) => {
  return (
    <>
      <>
        {Object.keys(disabledLayers).map((layerName) => (
          <label key={layerName}>
            <input
              type="checkbox"
              checked={!disabledLayers[layerName]}
              onChange={() => {
                const updatedLayers = { ...disabledLayers };
                updatedLayers[layerName] = !updatedLayers[layerName];
                AppEvents.disabledLayers.next(updatedLayers);
              }}
            />
            {layerName}
          </label>
        ))}
      </>

      {children}
    </>
  );
};
