import FullscreenIcon from '@mui/icons-material/Fullscreen';
import LayersIcon from '@mui/icons-material/Layers';
import ZoomInIcon from '@mui/icons-material/ZoomIn';
import ZoomOutIcon from '@mui/icons-material/ZoomOut';
import {
  Box,
  Checkbox,
  FormControl,
  FormControlLabel,
  FormGroup,
  IconButton,
  MenuItem,
  TextField,
} from '@mui/material';
import { Level } from 'api-client';
import { ChangeEvent } from 'react';
import React from 'react';

import { AppEvents } from '../app-events';

interface LayersControllerProps {
  disabledLayers: Record<string, boolean>;
  onChange: (event: ChangeEvent<HTMLInputElement>, value: string) => void;
  levels: Level[];
  currentLevel: Level;
  handleFullView: () => void;
  handleZoomIn: () => void;
  handleZoomOut: () => void;
}

export const LayersController = ({
  disabledLayers,
  onChange,
  levels,
  currentLevel,
  handleFullView,
  handleZoomIn,
  handleZoomOut,
}: LayersControllerProps) => {
  const [isHovered, setIsHovered] = React.useState(false);

  return (
    <Box
      sx={{
        position: 'absolute',
        top: '60px',
        left: '4px',
        width: 'auto',
        height: 'auto',
        zIndex: '1',
      }}
    >
      <FormControl>
        <TextField
          select
          id="level-select"
          label="Levels"
          variant="outlined"
          fullWidth
          margin="none"
          value={currentLevel.name}
          size="small"
          sx={{ width: '80px' }}
          onChange={(e: ChangeEvent<HTMLInputElement>) => onChange(e, e.target.value as string)}
        >
          {levels.map((level, i) => (
            <MenuItem key={i} value={level.name}>
              {level.name}
            </MenuItem>
          ))}
        </TextField>
      </FormControl>
      <div>
        <IconButton
          size="small"
          onClick={handleFullView}
          data-testid="full-view"
          sx={{ padding: 2 }}
        >
          <FullscreenIcon fontSize="large" />
        </IconButton>
      </div>
      <div>
        <IconButton size="small" onClick={handleZoomIn} data-testid="zoom-in" sx={{ padding: 2 }}>
          <ZoomInIcon fontSize="large" />
        </IconButton>
      </div>
      <div>
        <IconButton size="small" onClick={handleZoomOut} data-testid="zoom-out" sx={{ padding: 2 }}>
          <ZoomOutIcon fontSize="large" />
        </IconButton>
      </div>
      <div onMouseEnter={() => setIsHovered(true)} onMouseLeave={() => setIsHovered(false)}>
        <IconButton size="small" data-testid="layers" sx={{ padding: 2 }}>
          <LayersIcon fontSize="large" />
        </IconButton>
        {isHovered && (
          <div>
            {Object.keys(disabledLayers).map((layerName) => (
              <FormGroup key={layerName}>
                <FormControlLabel
                  control={
                    <Checkbox
                      sx={{ padding: 1.5 }}
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
                  sx={{ margin: '0' }}
                />
              </FormGroup>
            ))}
          </div>
        )}
      </div>
    </Box>
  );
};
