Page({
  data: {
    // 静态固定缺陷位置数组，与截图完全对应
    pos_defect: ["83"],    // crack裂缝 fifth 104
    pos_glue: [],  // glue胶水 second62 third69
    pos_dust: [],           // dirt灰尘 无
    pos_oil: ["52"],        // oil油污 forth77
    pos_wear: [],           // wear磨损 无
    pos_blackline: ["23"],  // line黑线 first54
    pos_none: [],           // 无缺陷
    totalNum: 3             // 固定总数5
  },
  // 无需云端配置、定时器、请求方法，全部删除
  onLoad() {
    // 无需拉取数据，data已静态写死
  },
  onUnload() {
    // 无定时器，无需清除
  },

  // 跳转统计页逻辑完全保留，无需改动
  goStatPage() {
    const { pos_defect, pos_glue, pos_dust, pos_oil, pos_wear, pos_blackline, pos_none } = this.data;
    const encode = (name, arr) => arr.map(num => `${name}|${num}`).join(",");
    wx.navigateTo({
      url: `/pages/stat/stat?d1=${encode("裂缝", pos_defect)}&d2=${encode("胶水", pos_glue)}&d3=${encode("灰尘", pos_dust)}&d4=${encode("油污", pos_oil)}&d5=${encode("磨损", pos_wear)}&d6=${encode("黑线", pos_blackline)}&d7=${encode("无缺陷", pos_none)}`
    })
  }
})