import { ChangeEvent } from 'react';
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
        <FormLabel id="demo-radio-buttons-group-label">Levels</FormLabel>
        {levels.map((level, i) => {
          const { name } = level;
          return (
            <RadioGroup
              aria-labelledby="demo-radio-buttons-group-label"
              value={currentLevel.name}
              name="radio-buttons-group"
              onChange={onClick}
              key={i}
            >
              <FormControlLabel value={name} control={<Radio />} label={name} />
            </RadioGroup>
          );
        })}
        {Object.keys(disabledLayers).map((layerName) => (
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
        ))}
      </div>
    </>
  );
};
