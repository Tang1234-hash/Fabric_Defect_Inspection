Page({
  data: { sortAllPos: [] },
  onLoad(options) {
    const allData = [];
    const paramList = [options.d1, options.d2, options.d3, options.d4, options.d5, options.d6, options.d7];

    paramList.forEach(paramStr => {
      if (!paramStr || paramStr === "") return;
      // 拆分每条缺陷
      const itemList = paramStr.split(",");
      itemList.forEach(item => {
        if (!item || item.indexOf("|") === -1) return;
        const [type, posText] = item.split("|");
        const posNum = Number(posText.trim());
        // 仅保留合法数字
        if (!isNaN(posNum) && posNum > 0) {
          allData.push({ type: type, pos: posNum });
        }
      })
    })
    // 按位置升序
    allData.sort((a, b) => a.pos - b.pos);
    this.setData({ sortAllPos: allData });
  },
  goBack() {
    wx.navigateBack();
  }
})