import { materialRenderers } from '@jsonforms/material-renderers';
import { JsonForms } from '@jsonforms/react';
import {
  Box,
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  Divider,
  Typography,
} from '@mui/material';
import React, { useState } from 'react';

export interface FlexibleTaskFormProps {
  open: boolean;
  onClose: () => void;
  onSuccess: (taskRequestJson: Record<string, unknown>) => void;
}

const getNestedValue = (obj: any, path: string): any => {
  return path
    .split('.')
    .reduce((acc, part) => (acc && acc[part] !== undefined ? acc[part] : undefined), obj);
};

export const resolveTemplate = (template: any, data: any): any => {
  if (typeof template === 'string') {
    if (template === '{now}') {
      return Date.now();
    }
    const exactMatch = template.match(/^{([a-zA-Z0-9_.]+)}$/);
    if (exactMatch) {
      if (exactMatch[1] === 'now') return Date.now();
      const val = getNestedValue(data, exactMatch[1]);
      return val !== undefined ? val : template;
    }
    return template.replace(/{([a-zA-Z0-9_.]+)}/g, (match, path) => {
      if (path === 'now') return Date.now().toString();
      const val = getNestedValue(data, path);
      return val !== undefined ? String(val) : match;
    });
  }

  if (Array.isArray(template)) {
    const result: any[] = [];
    for (const item of template) {
      if (item && typeof item === 'object' && item.action === '___forEach___') {
        const list = getNestedValue(data, item.source) || [];
        if (Array.isArray(list)) {
          for (const listItem of list) {
            const resolvedItem = resolveTemplate(item.template, { ...data, item: listItem });
            if (Array.isArray(resolvedItem)) {
              result.push(...resolvedItem);
            } else {
              result.push(resolvedItem);
            }
          }
        }
      } else {
        result.push(resolveTemplate(item, data));
      }
    }
    return result;
  }

  if (typeof template === 'object' && template !== null) {
    const obj: any = {};
    for (const [k, v] of Object.entries(template)) {
      obj[k] = resolveTemplate(v, data);
    }
    return obj;
  }

  return template;
};

export const mapFormDataToRmfTask = (formData: any, matchingSchema: any) => {
  if (!matchingSchema) {
    return { ...formData };
  }
  return resolveTemplate(matchingSchema, formData);
};

export const FlexibleTaskForm = ({ open, onClose, onSuccess }: FlexibleTaskFormProps) => {
  const [jsonSchema, setJsonSchema] = useState<any>(null);

  const [matchingSchema, setMatchingSchema] = useState<any>(null);

  const [formData, setFormData] = useState<any>({});

  const handleSchemaUpload = (
    event: React.ChangeEvent<HTMLInputElement>,

    setter: React.Dispatch<React.SetStateAction<any>>,
  ) => {
    const file = event.target.files?.[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (e) => {
      try {
        const parsed = JSON.parse(e.target?.result as string);
        setter(parsed);
      } catch (err) {
        console.error('Failed to parse JSON file', err);
      }
    };
    reader.readAsText(file);
  };

  const handleSubmit = () => {
    // Map formData to RMF task json using matchingSchema
    // For now, Phase 1 says "just console the final generated RMF task json."
    if (matchingSchema) {
      console.debug('Applying matching schema:', matchingSchema);
    }

    const generatedTask = mapFormDataToRmfTask(formData, matchingSchema);

    console.log('Final Generated RMF Task JSON:', generatedTask);
    onSuccess(generatedTask);
  };

  return (
    <Dialog open={open} onClose={onClose} maxWidth="sm" fullWidth>
      <DialogTitle>Create Flexible Task</DialogTitle>
      <DialogContent>
        <Box display="flex" flexDirection="column" gap={2} my={2}>
          <Box>
            <Typography variant="subtitle2">
              1. Upload JSONForms Schema (and UI Schema if combined)
            </Typography>
            <input
              type="file"
              accept=".json"
              onChange={(e) => handleSchemaUpload(e, setJsonSchema)}
            />
          </Box>
          <Box>
            <Typography variant="subtitle2">2. Upload Task-JSONForms Matching Schema</Typography>
            <input
              type="file"
              accept=".json"
              onChange={(e) => handleSchemaUpload(e, setMatchingSchema)}
            />
          </Box>
        </Box>
        <Divider />
        <Box mt={2} minHeight={200}>
          {jsonSchema ? (
            <JsonForms
              schema={jsonSchema.schema || jsonSchema}
              uischema={jsonSchema.uischema}
              data={formData}
              renderers={materialRenderers}
              onChange={({ data }) => setFormData(data)}
            />
          ) : (
            <Typography color="textSecondary">
              Initially blank. Upload schemas to render the form.
            </Typography>
          )}
        </Box>
      </DialogContent>
      <DialogActions>
        <Button onClick={onClose}>Cancel</Button>
        <Button variant="contained" color="primary" onClick={handleSubmit} disabled={!jsonSchema}>
          Submit
        </Button>
      </DialogActions>
    </Dialog>
  );
};
