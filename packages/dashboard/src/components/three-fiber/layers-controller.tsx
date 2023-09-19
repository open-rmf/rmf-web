import { ChangeEvent } from 'react';
import { Level } from 'api-client';
import { AppEvents } from '../app-events';
import {
  Box,
  Checkbox,
  FormControlLabel,
  FormGroup,
  FormLabel,
  Radio,
  RadioGroup,
} from '@mui/material';

interface LayersControllerProps {
  disabledLayers: Record<string, boolean>;
  onChange: (event: ChangeEvent<HTMLInputElement>, value: string) => void;
  levels: Level[];
  currentLevel: Level;
}

export const LayersController = ({
  disabledLayers,
  onChange,
  levels,
  currentLevel,
}: LayersControllerProps) => {
  return (
    <Box
      component="div"
      sx={{
        position: 'absolute',
        top: '60px',
        left: '4px',
        width: 'auto',
        height: 'auto',
        zIndex: '1',
      }}
    >
      <FormLabel>Levels</FormLabel>
      {levels.map((level, i) => {
        const { name } = level;
        return (
          <RadioGroup
            value={currentLevel.name}
            name="radio-buttons-group"
            onChange={onChange}
            key={i}
          >
            <FormControlLabel value={name} control={<Radio size="small" />} label={name} />
          </RadioGroup>
        );
      })}
      <FormLabel>Layers</FormLabel>
      {Object.keys(disabledLayers).map((layerName) => (
        <FormGroup key={layerName}>
          <FormControlLabel
            control={
              <Checkbox
                size="small"
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
    </Box>
  );
};
