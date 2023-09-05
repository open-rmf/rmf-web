import React, { ChangeEvent } from 'react';
import { AppEvents } from './app-events';
import { Checkbox, FormControlLabel, FormGroup, FormLabel, Radio, RadioGroup } from '@mui/material';
import { Level } from 'api-client';

interface CanvasControllerProps {
  disabledLayers: Record<string, boolean>;
  onClick: (event: ChangeEvent<HTMLInputElement>, value: string) => void;
  levels: Level[];
  currentLevel: Level;
}

export const CanvasController = ({
  disabledLayers,
  onClick,
  levels,
  currentLevel,
}: CanvasControllerProps) => {
  return (
    <>
      <div id="annotationsPanel">
        <ul>
          {levels.map((level, i) => {
            const { name } = level;
            return (
              <li key={i}>
                <FormLabel id="demo-radio-buttons-group-label">Gender</FormLabel>
                <RadioGroup
                  aria-labelledby="demo-radio-buttons-group-label"
                  value={currentLevel.name}
                  name="radio-buttons-group"
                  onChange={onClick}
                >
                  <FormControlLabel value={name} control={<Radio />} label={name} />
                </RadioGroup>
              </li>
            );
          })}
          {Object.keys(disabledLayers).map((layerName) => (
            <li key={layerName}>
              <FormGroup key={layerName}>
                <FormControlLabel
                  control={
                    <Checkbox
                      checked={!disabledLayers[layerName]}
                      onChange={() => {
                        const updatedLayers = { ...disabledLayers };
                        updatedLayers[layerName] = !updatedLayers[layerName];
                        AppEvents.disabledLayers.next(updatedLayers);
                      }}
                    />
                  }
                  label={layerName}
                />
              </FormGroup>
            </li>
          ))}
        </ul>
      </div>
    </>
  );
};
