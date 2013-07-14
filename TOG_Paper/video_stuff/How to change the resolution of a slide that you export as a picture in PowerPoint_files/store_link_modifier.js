(function () {
    $(function () {
        $(".oneMscomFooterV3 a[href='http://store.microsoft.com/']").each(function () {
            var href = $(this).attr("href");
            href = href.substr(0, href.indexOf('?'));
            $(this).attr("href", href + "http://store.microsoft.com/?WT.mc_id=SMCMSCOM_ENUS_NAV_BUYALL");
        });

    });
})();
