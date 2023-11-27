import { ChangeEvent } from 'react';
import { Level } from 'api-client';
import { AppEvents } from '../app-events';
import React from 'react';
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
import LayersIcon from '@mui/icons-material/Layers';
import ZoomInIcon from '@mui/icons-material/ZoomIn';
import ZoomOutIcon from '@mui/icons-material/ZoomOut';

interface LayersControllerProps {
  disabledLayers: Record<string, boolean>;
  onChange: (event: ChangeEvent<HTMLInputElement>, value: string) => void;
  levels: Level[];
  currentLevel: Level;
  handleZoomIn: () => void;
  handleZoomOut: () => void;
}

export const LayersController = ({
  disabledLayers,
  onChange,
  levels,
  currentLevel,
  handleZoomIn,
  handleZoomOut,
}: LayersControllerProps) => {
  const [isHovered, setIsHovered] = React.useState(false);

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
      <FormControl>
        <TextField
          select
          id="level-select"
          label="Levels"
          variant="outlined"
          fullWidth
          margin="normal"
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
        <IconButton size="small" onClick={handleZoomIn} data-testid="zoom-in">
          <ZoomInIcon fontSize="large" />
        </IconButton>
      </div>
      <div>
        <IconButton size="small" onClick={handleZoomOut} data-testid="zoom-out">
          <ZoomOutIcon fontSize="large" />
        </IconButton>
      </div>
      <div onMouseEnter={() => setIsHovered(true)} onMouseLeave={() => setIsHovered(false)}>
        <IconButton size="small" data-testid="layers">
          <LayersIcon fontSize="large" />
        </IconButton>
        {isHovered && (
          <div>
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
          </div>
        )}
      </div>
    </Box>
  );
};
