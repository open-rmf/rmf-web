export function tableCellStyle(flexGrow: string) {
  const rowHeight = 31;
  return {
    flex: `${flexGrow} 0 0`,
    overflow: 'hidden',
    height: rowHeight,
  };
}
