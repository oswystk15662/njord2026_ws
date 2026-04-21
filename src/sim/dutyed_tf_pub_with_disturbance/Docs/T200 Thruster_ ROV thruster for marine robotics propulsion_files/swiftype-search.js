/**
 * @param {*} func
 * @param {*} timeout
 */
function debounce(func, timeout = 300){
    let timer;
    return (...args) => {
        clearTimeout(timer);
        timer = setTimeout(() => { func.apply(this, args); }, timeout);
    };
}

function sendAutocompleteRequest() {
    let query = document.getElementById('swiftypeSearchInput').value;
    hideAutoComplete();

    if (query.length > 2) {
        let postObj = {
            "engine_key": "yZxxaqyFboC7HN3DxsmX",
            "q": query,
            "fetch_fields": {
                "page": ["title", "post_title", "post_type", "content", "image", "featured_image", "url", "skuprice", "sku_price", "sku_price_content", "highlight", "product_short_description", "product_categories", "post_categories", "post_excerpt", "_score", "revision_history", "variations"]
            },
            "sort_field": {
                "page": ["_score"]
            },
            "sort_direction": {
                "page": "desc"
            },
            "per_page": 50
        };
        let post = JSON.stringify(postObj);

        const url = "https://search-api.swiftype.com/api/v1/public/engines/search.json";
        let xhr = new XMLHttpRequest();

        xhr.open('POST', url, true);
        xhr.setRequestHeader('Content-type', 'application/json');
        xhr.send(post);

        xhr.onload = function () {
            if(xhr.status === 200) {
                let data = JSON.parse(xhr.response);
                showAutoComplete();
                populateAutocomplete(data);
            }
        }
    }
};

function sendSearchRequest(page = 1) {
    let query = document.getElementById('swiftypeSearchInput').value;
    let swiftSearchResults = document.getElementById('swiftSearchResults');
    document.getElementById('swiftSearchResultsEmpty').style.display = "unset";
    document.getElementById('swiftSearchResults').style.display = "none";
    swiftSearchResults.innerHTML = "";

    if (query.length > 2) {
        let postObj = {
            "engine_key": "yZxxaqyFboC7HN3DxsmX",
            "q": query,
            "fetch_fields": {
                "page": ["title", "post_title", "post_type", "content", "image", "featured_image", "url", "skuprice", "sku_price", "sku_price_content", "highlight", "revision_history", "variations"]
            },
            "sort_field": {
                "page": ["_score"]
            },
            "sort_direction": {
                "page": "desc"
            },
            "page": page,
            "per_page": 10
        };
        let post = JSON.stringify(postObj);

        const url = "https://search-api.swiftype.com/api/v1/public/engines/search.json";
        let xhr = new XMLHttpRequest();

        xhr.open('POST', url, true);
        xhr.setRequestHeader('Content-type', 'application/json');
        xhr.send(post);

        xhr.onload = function () {
            if(xhr.status === 200) {
                let data = JSON.parse(xhr.response);

                populateSearchResults(data);
            }
        }
    }
};

const swiftypeAutocomplete = debounce(() => sendAutocompleteRequest(), 300);
const swiftypeSearch = debounce((page) => sendSearchRequest(page), 200);

/**
 * Assigning event listeners.
 */
document.getElementById('swiftypeSearchInput').addEventListener("keyup", function (e) {
    let searchResultsSearchInput = document.getElementById('searchResultsSearchInput');

    if (e.target.value != searchResultsSearchInput.value) {
        searchResultsSearchInput.value = e.target.value;
        swiftypeAutocomplete();
    }
});
document.getElementById('swiftypeSearchInputMobile').addEventListener("input", function (e) {
    let searchResultsSearchInput = document.getElementById('searchResultsSearchInput');
    let swiftypeSearchInput = document.getElementById('swiftypeSearchInput');

    if (e.target.value != searchResultsSearchInput.value) {
        searchResultsSearchInput.value = e.target.value;
        swiftypeSearchInput.value = e.target.value;
        swiftypeAutocomplete();
    }
});
document.getElementById('searchResultsSearchInput').addEventListener("keyup", function (e) {
    let swiftypeSearchInput = document.getElementById('swiftypeSearchInput');
    let swiftypeSearchInputMobile = document.getElementById('swiftypeSearchInputMobile');

    if (e.target.value != swiftypeSearchInput.value) {
        swiftypeSearchInput.value = e.target.value;
        swiftypeSearchInputMobile.value = e.target.value;
        // swiftypeAutocomplete();
    }
});
document.getElementById('swiftypeSearchInput').addEventListener("focus", swiftypeAutocomplete);
document.getElementById('swiftypeSearchInput').addEventListener("click", swiftypeAutocomplete);
document.getElementById('swiftypeSearchInputMobile').addEventListener("focus", swiftypeAutocomplete);
document.getElementById('swiftypeSearchInputMobile').addEventListener("click", swiftypeAutocomplete);
document.getElementById('swiftypeSearchInput').addEventListener("click", function (e) { e.stopPropagation(); });
document.getElementById('swiftypeSearchInputForm').addEventListener("submit", function (e) {
    e.preventDefault();
    swiftypeSearch();
    showSearchResults();
});
document.getElementById('swiftypeSearchInputFormMobile').addEventListener("submit", function (e) {
    e.preventDefault();
    swiftypeSearch();
    showSearchResults();
});
document.getElementById('searchResultsSearchInputForm').addEventListener("submit", function (e) {
    e.preventDefault();
    swiftypeSearch();
    showSearchResults();
});
window.addEventListener("click", function() { hideAutoComplete(); });
document.getElementById('swiftSearchResultsOverlay').addEventListener("click", function(e) {
    e.target.style.display = "none";
    hideSearchResults();
});
document.getElementById("toggleSwiftSearchMobile").addEventListener("click", function (e) {
    let swiftypeSearchMobileContainer = document.getElementById("swiftypeSearchMobileContainer");
    swiftypeSearchMobileContainer.style.display = swiftypeSearchMobileContainer.style.display == "none" ? "unset" : "none";
});

function findKeywordContext(sentence, keyword) {
    var regex = new RegExp('([^\\s]*' + keyword + '[^\\s]*)', 'gi');
    try {
        var match = sentence.match(regex);

        if (match && match[0]) {
            if (match.length > 3) {
                return match.splice(1, 3).join(', ');
            }
            return match[0];
        }
    } catch (error) {}

    return false; // No match found
}

/**
 *
 * @param {*} data
 * @param {*} group
 */
function populateAutocomplete(data) {
    let swiftAutocomplete = document.getElementById('swiftAutocomplete');
    let swiftAutocompleteMobile = document.getElementById('swiftAutocompleteMobile');
    swiftAutocomplete.innerHTML = "";

    if (data.record_count == 0) {
        let noAvailableResult = document.createElement('div');
        noAvailableResult.innerHTML = `<div class="autocomplete-no-result"><span>No available result for “<strong>${ data.info.page.query }</strong>”</span></div>`;
        swiftAutocomplete.appendChild(noAvailableResult);
        swiftAutocompleteMobile.appendChild(noAvailableResult);

        return;
    }

    let postCategoriesJson = [];
    let productCategoriesJson = [];

    Object.entries(processSearchResults(data.records.page))
        .forEach(([key, { label, results}]) => {
            if (results && results.length > 0) {
                let groupSeparator = document.createElement('div');
                groupSeparator.innerHTML = `<div class="autocomplete-separator">${ label }</div>`;
                swiftAutocomplete.appendChild(groupSeparator);

                results.forEach((entry) => {
                    let containerLink = document.createElement("a");
                    let image = `<span class="st-ui-thumbnail" style="background-image: url(${ entry.image })"> <img src="${ entry.image }" alt="${ entry.post_title }"> </span>`;
                    let heading = `<span class="st-ui-type-heading">${ (entry.highlight.post_title || entry.highlight.title) || (entry.post_title || entry.title) }</span>`;
                    let skuPrice = `<span class="st-ui-type-detail">${ entry.sku_price || entry.skuprice }</span>`;
                    let detail = `<p class="st-ui-type-detail">${ (entry.highlight.content || entry.content) || "" }</p>`;

                    if (key === "products" && (entry.highlight.product_short_description || entry.product_short_description)) {
                        image = `<span class="st-ui-thumbnail" style="background-image: url(${ entry.featured_image || entry.image })"> <img src="${ entry.featured_image || entry.image }" alt="${ entry.post_title }"> </span>`;
                        detail = `<p class="st-ui-type-detail">${ entry.highlight.product_short_description || entry.product_short_description }</p>`;
                    } else if(key === "learn" && (entry.highlight.post_excerpt || entry.post_excerpt)) {
                        image = `<span class="st-ui-thumbnail" style="background-image: url(${ entry.featured_image || entry.image })"> <img src="${ entry.featured_image || entry.image }" alt="${ entry.post_title }"> </span>`;
                        detail = `<p class="st-ui-type-detail">${ entry.highlight.post_excerpt || entry.post_excerpt }</p>`;
                    }

                    // const variationMatch = findKeywordContext(entry.variations, data.info.page.query);
                    // const historyMatch = findKeywordContext(entry.revision_history, data.info.page.query);

                    // if (variationMatch) {
                    //     detail = detail + `<p class="st-ui-type-detail">Variation: <strong>${variationMatch}</strong></p>`;
                    // }
                    // if (historyMatch) {
                    //     detail = detail + `<p class="st-ui-type-detail">Revision History: <strong>${historyMatch}</strong></p>`;
                    // }

                    containerLink.href = entry.url;
                    containerLink.className = "st-ui-result st-ui-image __swiftype_result";
		    containerLink.innerHTML = '<div class="entry-containers"><div>' + image + '</div><div>' + heading + ((entry.sku_price || entry.skuprice) ? skuPrice : '') + detail + '</div>';
                    containerLink.setAttribute("data-score", entry._score);

                    swiftAutocomplete.appendChild(containerLink);

                    // Extract categories from results.
                    if (entry.post_categories) {
                        if (key === "products" && productCategoriesJson.indexOf(entry.post_categories) < 0 && postCategoriesJson.indexOf(entry.post_categories) < 0) {
                            productCategoriesJson.push(entry.post_categories);
                        } else if (key === "posts" && postCategoriesJson.indexOf(entry.post_categories) < 0 && productCategoriesJson.indexOf(entry.post_categories) < 0) {
                            postCategoriesJson.push(entry.post_categories);
                        }
                    }
                });
            }
        });

    let categoryLinkURLs = [];
    // console.log(postCategories, productCategories);

    // Search post categories.
    for (let i = 0; i < postCategoriesJson.length; i++) {
        const postCategories = JSON.parse(postCategoriesJson[i]);

        for (let ii = 0; ii < postCategories.length; ii++) {
            const postCategory = postCategories[ii];
            // console.log(postCategory, data.info.page.query, postCategory.name.toLocaleLowerCase().indexOf(data.info.page.query.toLocaleLowerCase()) >= 0);

            if (postCategory.name.toLocaleLowerCase().indexOf(data.info.page.query.toLocaleLowerCase()) >= 0) {
                let hasCategory = false;

                for (let iii = 0; iii < categoryLinkURLs.length; iii++) {
                    const categoryLinkURL = categoryLinkURLs[iii];

                    if (categoryLinkURL.label == postCategory.name) {
                        hasCategory = true;
                    }
                }

                if (!hasCategory) {
                    categoryLinkURLs.push({
                        label: postCategory.name,
                        url: postCategory.path,
                    });
                }
            }
        }
    }

    // Search product categories.
    for (let i = 0; i < productCategoriesJson.length; i++) {
        const productCategories = JSON.parse(productCategoriesJson[i]);

        for (let ii = 0; ii < productCategories.length; ii++) {
            const productCategory = productCategories[ii];
            // console.log(productCategory, data.info.page.query, productCategory.name.toLocaleLowerCase().indexOf(data.info.page.query.toLocaleLowerCase()) >= 0);

            if (productCategory.name.toLocaleLowerCase().indexOf(data.info.page.query.toLocaleLowerCase()) >= 0) {
                let hasCategory = false;

                for (let iii = 0; iii < categoryLinkURLs.length; iii++) {
                    const categoryLinkURL = categoryLinkURLs[iii];

                    if (categoryLinkURL.label == productCategory.name) {
                        hasCategory = true;
                    }
                }

                if (!hasCategory) {
                    categoryLinkURLs.push({
                        label: productCategory.name,
                        url: productCategory.path,
                    });
                }
            }
        }
    }

    // console.log(categoryLinkURLs);

    // Sort categories by label.
    categoryLinkURLs = categoryLinkURLs.sort((a, b) => {
        return a.label.localeCompare(b.label);
    });

    // Prepend category results to the autocomplete.
    if (categoryLinkURLs.length > 0) {
        let categorySeparator = "<div><div class=\"autocomplete-separator\">Categories</div></div>";
        let categoryLinks = "";

        for (let i = 0; i < categoryLinkURLs.length; i++) {
            let categoryLink = `<a href="${ categoryLinkURLs[i].url }" class="st-ui-result __swiftype_result"><span class="st-ui-type-heading swiftype-capitalize">${ categoryLinkURLs[i].label }</span></a>`;

            categoryLinks = categoryLinks + categoryLink;
        }

        swiftAutocomplete.innerHTML = categorySeparator + categoryLinks + swiftAutocomplete.innerHTML;
    }

    let viewMore = document.createElement('div');
    viewMore.innerHTML = "<div class=\"autocomplete-view-more\"><span>View More</span><span>>></span></div>";
    swiftAutocomplete.appendChild(viewMore);

    swiftAutocompleteMobile.innerHTML = swiftAutocomplete.innerHTML;

    let autocompleteViewMore = document.getElementsByClassName("autocomplete-view-more");

    let autocompleteViewMoreClickFunction = function() {
        swiftypeSearch();
        showSearchResults();
    };

    for (let i = 0; i < autocompleteViewMore.length; i++) {
        autocompleteViewMore[i].addEventListener("click", autocompleteViewMoreClickFunction, false);
    }
};

/**
 * The priority order of object_types is: product, learn, post, br_distributor, br_service_provider
 *
 * @param {*} data
 */
function processSearchResults(data) {
    let groupedSearchResults = {
        products: {
            label: "Products",
            results: [],
        },
        learn: {
            label: "Learn",
            results: [],
        },
        posts: {
            label: "Stories",
            results: [],
        },
        distributor: {
            label: "Distributor",
            results: [],
        },
        serviceProvider: {
            label: "Service Provider",
            results: [],
        },
    };

    for (let index = 0; index < data.length; index++) {
        const datum = data[index];
        const notPostUrls = [
            "store",
            "learn",
            "guide-tag",
            "category",
            "product-category",
            "distributors",
            "service-providers",
        ];

        if (datum.post_type == "product" && groupedSearchResults.products.results.length < 5) {
            groupedSearchResults.products.results.push(datum);
        } else if (datum.url.split('/')[3] == "learn" && groupedSearchResults.learn.results.length < 5) {
            groupedSearchResults.learn.results.push(datum);
        } else if (datum.url.split('/')[3] == "distributors" && groupedSearchResults.distributor.results.length < 5) {
            groupedSearchResults.distributor.results.push(datum);
        } else if (datum.url.split('/')[3] == "service-providers" && groupedSearchResults.serviceProvider.results.length < 5) {
            groupedSearchResults.serviceProvider.results.push(datum);
        } else if (datum.post_type == "article" && notPostUrls.indexOf(datum.url.split('/')[3]) < 0 && groupedSearchResults.posts.results.length < 5) {
            groupedSearchResults.posts.results.push(datum);
        }
    }

    return groupedSearchResults;
};

function populateSearchResults(data) {
    let swiftSearchResultsSummary = document.getElementById('swiftSearchResultsSummary');
    let swiftSearchResultsPaginationPagesContainer = document.getElementById('swiftSearchResultsPaginationPagesContainer');
    swiftSearchResultsPaginationPagesContainer.innerHTML = "";

    if (data.record_count == 0) {
        let summary = `<div class="st-query-present"> No available result for “<strong>${ data.info.page.query }</strong>”</div>`;

        swiftSearchResultsSummary.innerHTML = summary;

        return;
    }

    let swiftSearchResults = document.getElementById('swiftSearchResults');

    document.getElementById('swiftSearchResultsEmpty').style.display = "none";
    document.getElementById('swiftSearchResults').style.display = "unset";

    data.records.page.forEach((result) => {
        let containerLink = document.createElement("a");
        let image = `<span class="st-ui-thumbnail" style="background-image: url(${ result.featured_image || result.image })" title="${ result.post_title }"> <img src="${ result.featured_image || result.image }" title="${ result.post_title }"> </span>`;
        let heading = `<span class="st-ui-type-heading">${ (result.highlight.post_title || result.highlight.title) || (result.post_title || result.title) }</span>`;
        let detail = `<span class="st-ui-type-detail">${ (result.highlight.sku_price_content || result.sku_price_content) || (result.highlight.content || result.content) || "" }</span>`;

        // const variationMatch = findKeywordContext(result.variations, data.info.page.query);
        // const historyMatch = findKeywordContext(result.revision_history, data.info.page.query);

        // if (variationMatch) {
        //     detail = detail + `<p class="st-ui-type-detail">Variation: <strong>${variationMatch}</strong></p>`;
        // }
        // if (historyMatch) {
        //     detail = detail + `<p class="st-ui-type-detail">Revision History: <strong>${historyMatch}</strong></p>`;
        // }

        containerLink.href = result.url;
        containerLink.className = "st-ui-result st-ui-image __swiftype_result";
        containerLink.innerHTML = image + heading + detail;

        swiftSearchResults.appendChild(containerLink);
    });

    let summary = `<div class="st-query-present"> Showing <strong>${ (data.info.page.per_page * data.info.page.current_page) - 9 }</strong>–<strong>${ (data.info.page.per_page * data.info.page.current_page) > data.info.page.total_result_count ? data.info.page.total_result_count : data.info.page.per_page * data.info.page.current_page }</strong> of <strong>${ data.info.page.total_result_count }</strong> <span class="st-ui-search-summary-query"> for “${ data.info.page.query }” </span> </div>`;

    swiftSearchResultsSummary.innerHTML = summary;

    let min = (data.info.page.current_page - 5) < 1 ? 1 : (data.info.page.current_page - 5);
    let max = data.info.page.current_page < 6 ? 10 : min + 9;
    let last = max > data.info.page.num_pages ? data.info.page.num_pages : max;
    let first = max < (data.info.page.current_page + 5) ? last - (9 > data.info.page.num_pages ? data.info.page.num_pages : 9) : min;


    let prev = document.createElement("a");
    prev.className = `st-ui-pagination-link st-result-pagination-link ${ data.info.page.current_page < 2 ? "active" : "" }`;
    prev.innerHTML = `<span class="st-ui-arrow left-arrow ${ data.info.page.current_page < 2 ? "active" : "" }"></span> prev`;
    swiftSearchResultsPaginationPagesContainer.appendChild(prev);

    if (data.info.page.current_page > 1) {
        prev.addEventListener("click", function () {
            sendSearchRequest(data.info.page.current_page - 1);
        });
    }

    for (let page = first; page <= last; page++) {
        let pageLink = document.createElement("a");

        pageLink.href = "javascript:void(0);";
        pageLink.innerHTML = page;
        pageLink.className = `st-ui-pagination-link st-ui-pagination-number-link st-result-pagination-link ${ data.info.page.current_page == page ? "active" : "" }`;
        pageLink.addEventListener("click", function () {
            sendSearchRequest(page);
        });

        swiftSearchResultsPaginationPagesContainer.appendChild(pageLink);
    }

    let next = document.createElement("a");
    next.className = `st-ui-pagination-link st-result-pagination-link ${ data.info.page.num_pages <= data.info.page.current_page ? "active" : "" }`;
    next.innerHTML = `next <span class="st-ui-arrow right-arrow ${ data.info.page.num_pages <= data.info.page.current_page ? "active" : "" }"></span>`;
    swiftSearchResultsPaginationPagesContainer.appendChild(next);

    if (data.info.page.num_pages > data.info.page.current_page) {
        next.addEventListener("click", function () {
            sendSearchRequest(data.info.page.current_page + 1);
        });
    }
}

function showAutoComplete() {
    let query = document.getElementById('swiftypeSearchInput').value;

    if (query.length > 2 && isSearchResultsHidden()) {
        document.getElementById('swiftAutocompleteContainer').style.display = "block";
        document.getElementById('swiftAutocompleteContainerMobile').style.display = "block";
    }
}

function hideAutoComplete() {
    document.getElementById('swiftAutocompleteContainer').style.display = "none";
    document.getElementById('swiftAutocompleteContainerMobile').style.display = "none";
}

function hideSearchResults() {
    document.getElementById('swiftSearchResultsOverlay').style.display = "none";
    document.getElementById('swiftSearchResultsContainer').style.display = "none";
    document.documentElement.classList.toggle('noscroll', false);
}

function showSearchResults() {
    let query = document.getElementById('swiftypeSearchInput').value;

    if (query.length > 2) {
        hideAutoComplete();
        document.getElementById('swiftSearchResultsOverlay').style.display = "unset";
        document.getElementById('swiftSearchResultsContainer').style.display = "unset";
        document.documentElement.classList.toggle('noscroll', true);
    }
}

function isSearchResultsHidden() {
    return document.getElementById('swiftSearchResultsContainer').style.display == "none";
}

