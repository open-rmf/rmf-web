import React from 'react';

interface RenderSelectedComponentProps {
  selectedKey: string;
  obj: Record<string, React.ReactElement | null>;
}

export const RenderSelectedComponentFromDict = (
  props: RenderSelectedComponentProps,
): React.ReactElement => {
  const { obj, selectedKey } = props;

  const component = React.useMemo(() => {
    if (!Object.prototype.hasOwnProperty.call(obj, selectedKey)) {
      return null;
    }
    if (!obj[selectedKey]) {
      return null;
    }

    return obj[selectedKey];
  }, [selectedKey, obj]);

  return <>{component}</>;
};
