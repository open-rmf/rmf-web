export const mockOnClick = (): { onClick: () => void } => {
  return {
    onClick: () => {
      console.log('mock');
    },
  };
};

export const buildMockObject = (functionNames: string[]): Record<string, () => void> => {
  const newMockObject: Record<string, () => void> = {};
  functionNames.forEach((element) => {
    newMockObject[element] = () => console.log('mock');
  });
  return newMockObject;
};
